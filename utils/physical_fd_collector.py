#!/usr/bin/env python3
"""ROS1 physical finite-difference acquisition for offline_GANs_Train.py.

Consumes the atomic JSON exchange protocol. The collector is the sole
publisher of MAVROS attitude setpoints: it forwards offline_fd/baseline_attitude
and temporarily substitutes a requested probe while a fresh, synchronized
IMU/odometry response is acquired. It never arms or changes flight mode.
A baseline controller must continuously publish to the baseline topic, and
offline_fd/environment must report the actual fan configuration. Requests
wait for their state to recur; failure to meet a state/time/command condition
produces an explicit error receipt, never a simulated measurement.
"""
import argparse
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np

from export_flight_records import (net_acceleration_label, quaternion_inverse,
                                   quaternion_multiply, quaternion_normalize, rotate)

PROTOCOL = 'uadl.physical_fd.v1'
ENVIRONMENTS = ('fans_off', 'fan_a', 'fans_on')


def rotation_quaternion(r):
    """Unit quaternion (w,x,y,z) for an orthonormal body-to-world matrix."""
    trace = float(np.trace(r))
    if trace > 0:
        s = 2*math.sqrt(trace+1)
        q = (s/4, (r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s, (r[1, 0]-r[0, 1])/s)
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2*math.sqrt(1+r[i, i]-r[j, j]-r[k, k])
        xyz = [0.0, 0.0, 0.0]
        xyz[i], xyz[j], xyz[k] = s/4, (r[j, i]+r[i, j])/s, (r[k, i]+r[i, k])/s
        q = ((r[k, j]-r[j, k])/s, *xyz)
    return quaternion_normalize(q)


def map_command(command, yaw, voltage, config):
    """C++ mapAcceleration convention, rejecting every saturation/modification."""
    gravity, mass = float(config['gra']), float(config['mass'])
    model = config['thrust_model']
    hover = float(model['hover_percentage'])
    tilt = math.radians(float(config['max_angle']))
    u = np.asarray(command, dtype=np.float64)
    if u.shape != (3,) or not np.isfinite(u).all() or not 0 < hover <= 1 or \
            not math.isfinite(gravity) or not math.isfinite(mass) or min(gravity, mass) <= 0 or \
            not math.isfinite(yaw) or not 0 <= tilt < math.pi/2:
        raise ValueError('Invalid attitude/thrust mapping inputs')
    for j, axis in enumerate(('xy', 'xy', 'z')):
        limits = config['rtmpc'][axis]
        if not float(limits['limit_u_min']) <= u[j] <= float(limits['limit_u_max']):
            raise ValueError('Probe exceeds the configured physical input interval')
    force = u+np.array([0.0, 0.0, gravity])
    if force[2] <= 1e-8 or np.linalg.norm(force[:2]) > force[2]*math.tan(tilt):
        raise ValueError('Probe requires an invalid vertical thrust or tilt saturation')
    magnitude = float(np.linalg.norm(force))
    zb, xc = force/magnitude, np.array([math.cos(yaw), math.sin(yaw), 0.0])
    yb = np.cross(zb, xc)
    if np.linalg.norm(yb) < 1e-8:
        raise ValueError('Degenerate attitude mapping')
    yb /= np.linalg.norm(yb)
    q = rotation_quaternion(np.column_stack((np.cross(yb, zb), yb, zb)))
    if model['accurate_thrust_model']:
        k1, k2, k3 = float(model['K1']), float(model['K2']), float(model['K3'])
        if not all(map(math.isfinite, (voltage, k1, k2, k3))) or voltage <= 0 or k1 <= 0 or not 0 <= k3 <= 1:
            raise ValueError('Invalid voltage-dependent thrust model')
        scale, linear = k1*voltage**k2, 1-k3
        target = mass*magnitude/scale
        thrust = target if k3 < 1e-10 else 2*target/(linear+math.sqrt(linear**2+4*k3*target))
        if not 0 <= thrust <= 1:
            raise ValueError('Probe requires thrust saturation')
        thrust = float(np.float32(thrust))
        achieved = scale*(k3*thrust**2+linear*thrust)/mass
    else:
        thrust = magnitude*hover/gravity
        if not 0 <= thrust <= 1:
            raise ValueError('Probe requires thrust saturation')
        thrust = float(np.float32(thrust))
        achieved = thrust*gravity/hover
    return q, thrust, achieved*zb-np.array([0.0, 0.0, gravity])


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False))
    os.replace(temporary, path)


class Collector:
    def __init__(self, settings, controller_config):
        import rospy
        from geometry_msgs.msg import Quaternion
        from mavros_msgs.msg import AttitudeTarget, State
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import BatteryState, Imu
        from std_msgs.msg import String
        self.ros, self.AttitudeTarget, self.Quaternion = rospy, AttitudeTarget, Quaternion
        self.settings, self.config = settings, controller_config
        self.directory = Path(settings['exchange_directory']).expanduser().resolve()
        for name in ('requests', 'responses', 'cancellations'):
            (self.directory/name).mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.odom = self.imu = self.baseline = self.flight_state = None
        self.environment, self.voltage = '', math.nan
        self.battery_stamp = -math.inf
        self.active, self.last_failure = None, None
        self.sent, self.samples = deque(maxlen=500), deque(maxlen=500)
        self.sensor_timeout = float(settings.get('sensor_timeout', .5))
        self.baseline_timeout = float(settings.get('baseline_timeout', .1))
        self.response_delay = float(settings['response_delay'])
        if not 0 <= self.response_delay < float(settings['max_response_delay']):
            raise ValueError('response_delay must lie in [0,max_response_delay)')
        if min(self.sensor_timeout, self.baseline_timeout) <= 0:
            raise ValueError('Sensor and baseline timeouts must be positive')
        ns = settings.get('namespace', '/drone1').rstrip('/')
        self.publisher = rospy.Publisher(ns+'/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.subscribers = [
            rospy.Subscriber(ns+'/mavros/local_position/odom', Odometry, self.on_odom, queue_size=50),
            rospy.Subscriber(ns+'/mavros/imu/data', Imu, self.on_imu, queue_size=100),
            rospy.Subscriber(ns+'/mavros/battery', BatteryState, self.on_battery, queue_size=10),
            rospy.Subscriber(ns+'/mavros/state', State, self.on_flight_state, queue_size=10),
            rospy.Subscriber(ns+'/offline_fd/baseline_attitude', AttitudeTarget, self.on_baseline, queue_size=10),
            rospy.Subscriber(ns+'/offline_fd/environment', String, self.on_environment, queue_size=10)]
        self.timer = rospy.Timer(rospy.Duration(.01), self.publish)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        try:
            attitude = quaternion_normalize((q.w, q.x, q.y, q.z))
            state = np.asarray([p.x, p.y, p.z, *rotate(attitude, (v.x, v.y, v.z))])
            if not np.isfinite(state).all():
                return
        except ValueError:
            return
        with self.lock:
            self.odom = (msg.header.stamp.to_sec(), state, attitude)
            self.observe()

    def on_imu(self, msg):
        q, a = msg.orientation, msg.linear_acceleration
        try:
            attitude = quaternion_normalize((q.w, q.x, q.y, q.z))
            force = (a.x, a.y, a.z)
            if not all(map(math.isfinite, force)):
                return
        except ValueError:
            return
        with self.lock:
            self.imu = (msg.header.stamp.to_sec(), force, attitude)
            self.observe()

    def on_baseline(self, msg):
        with self.lock:
            self.baseline = (self.ros.Time.now().to_sec(), copy.deepcopy(msg))

    def on_environment(self, msg):
        with self.lock:
            self.environment = msg.data.strip()

    def on_flight_state(self, msg):
        with self.lock:
            self.flight_state = msg

    def on_battery(self, msg):
        with self.lock:
            self.voltage, self.battery_stamp = float(msg.voltage), msg.header.stamp.to_sec()

    def ready(self, now):
        return self.odom is not None and self.imu is not None and self.baseline is not None and \
            0 <= now-self.odom[0] <= self.sensor_timeout and 0 <= now-self.imu[0] <= self.sensor_timeout and \
            0 <= now-self.baseline[0] <= self.baseline_timeout and self.flight_state is not None and \
            self.flight_state.connected and self.flight_state.armed and self.flight_state.mode == 'OFFBOARD'

    def publish(self, _event):
        with self.lock:
            now = self.ros.Time.now().to_sec()
            if self.baseline is None or not 0 <= now-self.baseline[0] <= self.baseline_timeout:
                if self.active:
                    self.last_failure = 'Baseline command stream became stale'
                    self.active = None
                return
            output = copy.deepcopy(self.baseline[1])
            measurement = None
            try:
                if self.active:
                    if not self.ready(now):
                        raise ValueError('Vehicle/sensor state is unavailable during a physical probe')
                    if self.environment != self.active['environment']:
                        raise ValueError('Physical environment changed during a probe')
                    if self.config['thrust_model']['accurate_thrust_model'] and \
                            not 0 <= now-self.battery_stamp <= self.sensor_timeout:
                        raise ValueError('Battery voltage is stale')
                    q_odom, q_imu = self.odom[2], self.imu[2]
                    w, x, y, z = q_odom
                    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                    q_cmd, thrust, executed = map_command(self.active['command'], yaw, self.voltage, self.config)
                    if np.max(np.abs(executed-self.active['command'])) > self.active['action_tolerance']:
                        raise ValueError('Published float32 thrust changes the requested probe beyond tolerance')
                    q_sp = quaternion_normalize(quaternion_multiply(
                        quaternion_multiply(q_imu, quaternion_inverse(q_odom)), q_cmd))
                    output = self.AttitudeTarget()
                    output.type_mask = (self.AttitudeTarget.IGNORE_ROLL_RATE |
                                        self.AttitudeTarget.IGNORE_PITCH_RATE | self.AttitudeTarget.IGNORE_YAW_RATE)
                    output.orientation = self.Quaternion(x=q_sp[1], y=q_sp[2], z=q_sp[3], w=q_sp[0])
                    output.thrust = thrust
                    measurement = (self.active['probe_id'], executed.copy())
            except (ValueError, OverflowError, ZeroDivisionError) as exc:
                self.last_failure, self.active = str(exc), None
                output = copy.deepcopy(self.baseline[1])
            output.header.stamp = self.ros.Time.now()
            self.publisher.publish(output)
            if measurement is not None and self.active is not None and 'published' not in self.active:
                self.active['published'] = output.header.stamp.to_sec()
            self.sent.append((output.header.stamp.to_sec(), measurement))

    def observe(self):
        if self.active is None or self.odom is None or self.imu is None:
            return
        odom_stamp, state, q_odom = self.odom
        imu_stamp, force, _ = self.imu
        if abs(imu_stamp-odom_stamp) > self.active['max_sensor_skew']:
            return
        first, last = min(odom_stamp, imu_stamp), max(odom_stamp, imu_stamp)
        commands = [item for item in self.sent if item[0] <= first]
        if not commands or any(first < item[0] <= last for item in self.sent):
            return
        stamp, measurement = commands[-1]
        if measurement is None or measurement[0] != self.active['probe_id']:
            return
        if last-stamp > self.active['max_response_delay'] or \
                first < self.active.get('published', math.inf)+self.response_delay:
            return
        if (np.abs(state-self.active['state']) > self.active['state_tolerance']).any():
            return
        if self.samples and odom_stamp <= self.samples[-1]['response_stamp']:
            return
        self.samples.append({'probe_id': measurement[0], 'state': state.tolist(),
                             'executed_commands': measurement[1].tolist(),
                             'net_acceleration': list(net_acceleration_label(q_odom, force, self.config['gra'])),
                             'command_stamp': stamp, 'response_stamp': odom_stamp,
                             'imu_stamp': imu_stamp, 'odom_stamp': odom_stamp})

    def acquire(self, request):
        if request.get('protocol') != PROTOCOL or request.get('measurement_source') != 'physical_imu' or \
                request.get('probe_order') != ['minus', 'nominal', 'plus']:
            raise ValueError('Unsupported physical measurement request')
        state = np.asarray(request['state'], dtype=np.float64)
        commands = np.asarray(request['commands'], dtype=np.float64)
        tolerance = np.asarray(request['state_tolerance'], dtype=np.float64)
        if state.ndim != 2 or state.shape[1] != 6 or commands.shape != (len(state), 3, 3) or \
                tolerance.shape != (6,) or not np.isfinite(state).all() or not np.isfinite(commands).all() or \
                not np.isfinite(tolerance).all() or (tolerance < 0).any():
            raise ValueError('Malformed state, action or tolerance arrays')
        if len(request['environment']) != len(state) or any(env not in ENVIRONMENTS for env in request['environment']):
            raise ValueError('Unsupported environment labels')
        for key in ('max_probe_span', 'max_sensor_skew', 'max_response_delay', 'action_tolerance'):
            if not math.isfinite(float(request[key])) or float(request[key]) <= 0:
                raise ValueError('Invalid request tolerance: '+key)
        if self.response_delay >= float(request['max_response_delay']):
            raise ValueError('Requested response window is shorter than response_delay')
        fields = ('state', 'executed_commands', 'net_acceleration', 'command_stamp',
                  'response_stamp', 'imu_stamp', 'odom_stamp')
        result = {key: [] for key in fields}
        result.update({key: request[key] for key in ('protocol', 'request_id', 'measurement_source',
                                                   'probe_order', 'environment')})
        cancellation = self.directory/'cancellations'/(request['request_id']+'.json')
        deadline = time.monotonic()+float(self.settings['timeout_seconds'])
        try:
            for i in range(len(state)):
                triplet, triplet_started = [], None
                for probe in range(3):
                    probe_id, sample = (request['request_id'], i, probe), None
                    while sample is None:
                        if self.ros.is_shutdown() or cancellation.exists():
                            raise ValueError('Physical acquisition cancelled')
                        if time.monotonic() >= deadline:
                            raise TimeoutError('Requested state/environment did not recur before the acquisition deadline')
                        if triplet_started is not None and time.monotonic()-triplet_started > request['max_probe_span']:
                            raise TimeoutError('Repeated-state probe triplet exceeded max_probe_span')
                        with self.lock:
                            if self.last_failure:
                                raise ValueError(self.last_failure)
                            now = self.ros.Time.now().to_sec()
                            if self.active is None and self.ready(now) and self.environment == request['environment'][i] and \
                                    (np.abs(self.odom[1]-state[i]) <= tolerance).all():
                                if triplet_started is None:
                                    triplet_started = time.monotonic()
                                self.active = {'probe_id': probe_id, 'command': commands[i, probe],
                                               'state': state[i], 'state_tolerance': tolerance, 'started': now,
                                               'environment': request['environment'][i],
                                               'action_tolerance': float(request['action_tolerance']),
                                               'max_sensor_skew': float(request['max_sensor_skew']),
                                               'max_response_delay': float(request['max_response_delay'])}
                            sample = next((item for item in reversed(self.samples) if item['probe_id'] == probe_id), None)
                            if sample is not None:
                                self.active = None
                        if sample is None:
                            time.sleep(.001)
                    triplet.append(sample)
                for key in fields:
                    result[key].append([sample[key] for sample in triplet])
            return result
        finally:
            with self.lock:
                self.active, self.last_failure = None, None

    def run(self):
        while not self.ros.is_shutdown():
            for path in sorted((self.directory/'requests').glob('*.json')):
                response = self.directory/'responses'/path.name
                if response.exists() or (self.directory/'cancellations'/path.name).exists():
                    continue
                request = json.loads(path.read_text())
                if request.get('request_id') != path.stem:
                    raise ValueError('Request ID differs from its exchange filename')
                try:
                    receipt = self.acquire(request)
                except (ValueError, TimeoutError, KeyError, TypeError) as exc:
                    receipt = {'protocol': PROTOCOL, 'request_id': path.stem, 'error': str(exc)}
                    self.ros.logerr('Physical FD request %s failed: %s', path.stem, exc)
                atomic_json(response, receipt)
            time.sleep(.05)


def main():
    import rospy
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', required=True, help='same physical acquisition JSON as the trainer')
    parser.add_argument('--controller-config', required=True, help='px4ctrl YAML with mass, gravity and thrust model')
    args = parser.parse_args(rospy.myargv()[1:])
    settings = json.loads(Path(args.settings).read_text())
    config = yaml.safe_load(Path(args.controller_config).read_text())
    rospy.init_node('physical_fd_collector')
    Collector(settings, config).run()


if __name__ == '__main__':
    main()
