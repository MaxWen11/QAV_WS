#!/usr/bin/env python3
"""Explicitly started, synchronized two-vehicle trial protocol.

Run only after an operator establishes valid OFFBOARD hover on both vehicles,
then call this node's ~start_trial (std_srvs/Trigger). No arming, takeoff or mode
service is called here. One shared clock drives both references: 10 s warm-up,
controller reset reservations at a shared future epoch, 60 s scoring at 100 Hz,
smooth braking/hold. Both reset acknowledgements must arrive before that epoch.
Reservations start reset_lead_s before warm-up ends; scoring always starts at
start_time + 10 s, so service lead time does not extend the warm-up interval.
Invalid/stale state stops commands for BOTH vehicles; it never auto-resumes.
Metrics continue on the original scoring clock, including backup operation,
with missing samples reported as coverage gaps.

Reference: lemniscate with half-widths A=1.5 m (y) and B=1.0 m (x) and a 12 s
period, i.e. peak reference acceleration 3*A*(2*pi/T)^2 = 1.23 m/s^2 inside the
3 m/s^2 horizontal input range, and the formation offset r_des=[0,1.5,0] m
(Eq. 49). ROS parameters expose the shape, transition and stopping durations,
and the hover, synchronization and staleness limits. Both odometry streams use
the same inertial frame and synchronized clocks. Pure trajectory/protocol/metric
helpers import without ROS.
"""
import bisect
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time


@dataclass(frozen=True)
class TrialConfig:
    amplitude_a: float = 1.5
    amplitude_b: float = 1.0
    period: float = 12.0
    rate: float = 100.0
    warmup_s: float = 10.0
    scoring_s: float = 60.0
    startup_s: float = 1.5
    transition_s: float = 5.0
    stopping_s: float = 3.0
    separation: tuple = (0.0, 1.5, 0.0)
    yaw: float = -math.pi / 2

    def validate(self):
        values = [self.amplitude_a, self.amplitude_b, self.period, self.rate, self.warmup_s,
                  self.scoring_s, self.startup_s, self.transition_s, self.stopping_s, self.yaw]
        if not all(math.isfinite(v) for v in values) or min(self.period, self.rate, self.transition_s, self.stopping_s) <= 0:
            raise ValueError('Trajectory parameters must be finite and durations/rate positive')
        if self.transition_s > self.warmup_s or not 0 < self.startup_s <= self.scoring_s:
            raise ValueError('Transition must finish during warm-up; startup must fit scoring')


@dataclass(frozen=True)
class OdomSample:
    stamp: float
    position: tuple
    velocity: tuple
    frame: str = 'map'


def _multiply_derivatives(a, b):
    return [sum(math.comb(n, k) * a[k] * b[n-k] for k in range(n+1)) for n in range(4)]


def _divide_derivatives(a, b):
    result = []
    for n in range(4):
        result.append((a[n] - sum(math.comb(n, k) * b[k] * result[n-k] for k in range(1, n+1))) / b[0])
    return result


def figure8(t, config):
    """Analytic position, velocity, acceleration, jerk of the rational lemniscate."""
    w = 2 * math.pi / config.period
    s = [w**k * math.sin(w*t + k*math.pi/2) for k in range(4)]
    c = [w**k * math.cos(w*t + k*math.pi/2) for k in range(4)]
    denominator = _multiply_derivatives(s, s)
    denominator[0] += 1
    x = _divide_derivatives([config.amplitude_b * a for a in _multiply_derivatives(s, c)], denominator)
    y = _divide_derivatives([config.amplitude_a * a for a in c], denominator)
    return tuple((x[k], y[k], 0.0) for k in range(4))


def _blend_derivatives(t, duration):
    if t >= duration:
        return (1.0, 0.0, 0.0, 0.0)
    if t <= 0:
        return (0.0, 0.0, 0.0, 0.0)
    u = t / duration
    coefficients = {4: 35., 5: -84., 6: 70., 7: -20.}
    return tuple(sum(value * math.factorial(power) / math.factorial(power-n) * u**(power-n)
                     for power, value in coefficients.items() if power >= n) / duration**n for n in range(4))


def brake_phase(t, duration):
    """Phase travels forward and smoothly slows from unit speed to zero."""
    if t >= duration:
        return duration / 2, 0., 0., 0.
    u = max(0., t / duration)
    return (duration*(u - 2.5*u**4 + 3*u**5 - u**6),
            1 - 10*u**3 + 15*u**4 - 6*u**5,
            (-30*u**2 + 60*u**3 - 30*u**4)/duration,
            (-60*u + 180*u**2 - 120*u**3)/duration**2)


class TrialProtocol:
    def __init__(self, config):
        config.validate()
        self.config = config
        self.state = 'idle'
        self.start_time = None
        self.scoring_start = None
        self.reset_epoch = None
        self.initial_positions = None
        self.center = None
        self.abort_reason = None

    def start(self, now, initial_positions):
        if self.state != 'idle':
            raise ValueError('A trial can only be started once per process')
        if len(initial_positions) != 2 or any(len(p) != 3 or not all(map(math.isfinite, p)) for p in initial_positions):
            raise ValueError('Two finite hover positions are required')
        self.initial_positions = tuple(tuple(p) for p in initial_positions)
        start_curve = figure8(0., self.config)[0]
        self.center = tuple((initial_positions[0][k] + initial_positions[1][k])/2 - start_curve[k] for k in range(3))
        self.start_time = now
        self.state = 'warmup'

    def reset_due(self, now, reset_lead_s=1.0):
        if not math.isfinite(reset_lead_s) or not 0 < reset_lead_s <= self.config.warmup_s:
            raise ValueError('Reset lead must be positive and fit within warm-up')
        return self.state == 'warmup' and now >= self.start_time + self.config.warmup_s - reset_lead_s

    def begin_reset(self, now, reset_epoch, reset_lead_s=1.0):
        if not self.reset_due(now, reset_lead_s):
            raise ValueError('Reset requested before its warm-up reservation window')
        if not math.isfinite(reset_epoch) or reset_epoch <= now:
            raise ValueError('Reset epoch must be strictly in the future')
        expected_epoch = self.start_time + self.config.warmup_s
        if abs(reset_epoch - expected_epoch) > 1e-9:
            raise ValueError('Reset epoch must equal the end of the 10 s warm-up')
        self.reset_epoch = expected_epoch
        self.state = 'resetting'

    def acknowledge_resets(self, now, successes):
        if self.state != 'resetting':
            raise ValueError('Unexpected controller reset completion')
        if len(successes) != 2 or not all(successes) or now >= self.reset_epoch:
            self.abort('Controller reset rejected or acknowledged too late for the shared epoch')
            return False
        self.state = 'awaiting_epoch'
        return True

    def activate_scoring(self, now):
        if self.state == 'awaiting_epoch' and now >= self.reset_epoch:
            self.scoring_start = self.reset_epoch
            self.state = 'scoring'
            return True
        return False

    def abort(self, reason):
        self.abort_reason = reason
        self.state = 'aborted'

    def reference(self, now):
        if self.start_time is None:
            raise ValueError('Trial is not started')
        elapsed = max(0., now - self.start_time)
        phase = elapsed
        speed, acceleration, jerk = 1., 0., 0.
        if self.scoring_start is not None:
            stopping_start = self.scoring_start + self.config.scoring_s
            if now >= stopping_start:
                delta, speed, acceleration, jerk = brake_phase(now - stopping_start, self.config.stopping_s)
                phase = stopping_start - self.start_time + delta
        curve = figure8(phase, self.config)
        transformed = [curve[0], tuple(v*speed for v in curve[1]),
                       tuple(curve[2][k]*speed**2 + curve[1][k]*acceleration for k in range(3)),
                       tuple(curve[3][k]*speed**3 + 3*curve[2][k]*speed*acceleration + curve[1][k]*jerk for k in range(3))]
        blend = _blend_derivatives(elapsed, self.config.transition_s)
        outputs = []
        for vehicle in range(2):
            sign = 0.5 if vehicle == 0 else -0.5
            target = [tuple(transformed[0][k] + self.center[k] + sign*self.config.separation[k] for k in range(3))] + transformed[1:]
            result = []
            for n in range(4):
                derivative = []
                for axis in range(3):
                    value = self.initial_positions[vehicle][axis] if n == 0 else 0.0
                    for k in range(n+1):
                        term = target[n-k][axis]
                        if n-k == 0:
                            term -= self.initial_positions[vehicle][axis]
                        value += math.comb(n, k) * blend[k] * term
                    derivative.append(value)
                result.append(tuple(derivative))
            outputs.append(tuple(result))
        return tuple(outputs)


class TrialMetrics:
    """Unique, aligned sample bins; coverage and partial metrics remain explicit."""
    def __init__(self, protocol, max_skew_s):
        self.protocol = protocol
        self.max_skew_s = max_skew_s
        self.last_stamps = [-float('inf'), -float('inf')]
        self.records = {}

    def add(self, first, second):
        origin = self.protocol.scoring_start
        if origin is None or abs(first.stamp - second.stamp) > self.max_skew_s:
            return False
        if first.frame != second.frame or any(s.stamp <= old for s, old in zip((first, second), self.last_stamps)):
            return False
        common = (first.stamp + second.stamp)/2
        elapsed = common - origin
        config = self.protocol.config
        if any(s.stamp < origin or s.stamp >= origin + config.scoring_s for s in (first, second)):
            return False
        if elapsed < 0 or elapsed >= config.scoring_s:
            return False
        bin_id = int(math.floor(elapsed * config.rate + 1e-7))
        if bin_id in self.records:
            return False
        desired = (self.protocol.reference(first.stamp)[0][0], self.protocol.reference(second.stamp)[1][0])
        sync_error_sq = sum((first.position[k] - second.position[k] - config.separation[k])**2 for k in range(3))
        tracking_sq = [sum((sample.position[k] - desired[i][k])**2 for k in range(3))
                       for i, sample in enumerate((first, second))]
        self.records[bin_id] = {'t': elapsed, 'stamps': [first.stamp, second.stamp],
                                'positions': [first.position, second.position],
                                'desired_positions': desired,
                                'sync_error_sq': sync_error_sq, 'tracking_error_sq': tracking_sq}
        self.last_stamps = [first.stamp, second.stamp]
        return True

    def report(self, now):
        config = self.protocol.config
        expected = int(round(config.rate * config.scoring_s))
        startup_expected = int(round(config.rate * config.startup_s))
        records = list(self.records.values())
        startup = [r for r in records if r['t'] < config.startup_s]
        finished = self.protocol.scoring_start is not None and now >= self.protocol.scoring_start + config.scoring_s
        complete = finished and len(records) == expected
        startup_complete = len(startup) == startup_expected
        def rmse(values):
            return math.sqrt(sum(values)/len(values)) if values else None
        sync = rmse([r['sync_error_sq'] for r in records])
        tracking = [rmse([r['tracking_error_sq'][i] for r in records]) for i in range(2)]
        startup_rmse = rmse([r['sync_error_sq'] for r in startup])
        peak = math.sqrt(max(r['sync_error_sq'] for r in records)) if records else None
        return {'scoring_epoch': self.protocol.scoring_start, 'duration_s': config.scoring_s,
                'elapsed_scoring_s': min(config.scoring_s, max(0., now - self.protocol.scoring_start)) if self.protocol.scoring_start is not None else 0.,
                'scoring_duration_completed': finished, 'trial_interrupted': self.protocol.abort_reason,
                'valid_bins': len(records), 'expected_bins': expected, 'coverage': len(records)/expected,
                'startup_valid_bins': len(startup), 'startup_expected_bins': startup_expected,
                'startup_coverage': len(startup)/startup_expected,
                'full_trial_sync_rmse': sync if complete else None,
                'full_trial_tracking_rmse': tracking if complete else [None, None],
                'full_trial_peak_sync_error': peak if complete else None,
                'startup_sync_rmse': startup_rmse if startup_complete else None,
                'observed_sync_rmse': sync, 'observed_tracking_rmse': tracking,
                'observed_startup_sync_rmse': startup_rmse,
                'observed_peak_sync_error': peak,
                'complete_metrics': complete,
                'coverage_definition': 'unique paired odometry bins / expected 100 Hz scoring bins; no stale samples or interpolation'}


class DualDroneManager:
    def __init__(self):
        # ROS imports stay in the adapter so pure unit tests cannot start hardware.
        import rospy
        from nav_msgs.msg import Odometry
        from mavros_msgs.msg import State
        from quadrotor_msgs.msg import PositionCommand
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger, TriggerResponse
        self.rospy, self.PositionCommand = rospy, PositionCommand
        self.Trigger, self.TriggerResponse = Trigger, TriggerResponse
        self.lock = threading.RLock()
        self.names = rospy.get_param('~drone_names', ['drone1', 'drone2'])
        if len(self.names) != 2 or self.names[0] == self.names[1]:
            raise ValueError('Exactly two distinct drone names are required')
        self.config = TrialConfig(amplitude_a=float(rospy.get_param('~amplitude_a', 1.5)),
                                  amplitude_b=float(rospy.get_param('~amplitude_b', 1.0)),
                                  period=float(rospy.get_param('~period', 12.0)),
                                  transition_s=float(rospy.get_param('~transition_s', 5.0)),
                                  stopping_s=float(rospy.get_param('~stopping_s', 3.0)),
                                  yaw=float(rospy.get_param('~yaw', -math.pi/2)))
        self.protocol = TrialProtocol(self.config)
        self.frame = rospy.get_param('~reference_frame', 'map')
        self.odom_timeout = float(rospy.get_param('~odom_timeout_s', 0.1))
        self.state_timeout = float(rospy.get_param('~fcu_state_timeout_s', 1.0))
        self.ready_timeout = float(rospy.get_param('~controller_ready_timeout_s', 0.5))
        self.max_skew = float(rospy.get_param('~max_sync_skew_s', 0.01))
        self.hover_speed = float(rospy.get_param('~hover_speed_limit', 0.15))
        self.hover_window = float(rospy.get_param('~hover_stable_s', 1.0))
        self.reset_timeout = float(rospy.get_param('~reset_timeout_s', 3.0))
        self.reset_lead = float(rospy.get_param('~reset_lead_s', 1.0))
        limits = [self.odom_timeout, self.state_timeout, self.ready_timeout, self.hover_speed,
                  self.hover_window, self.reset_timeout, self.reset_lead]
        if not all(map(math.isfinite, limits + [self.max_skew])) or min(limits) <= 0 or self.max_skew < 0:
            raise ValueError('Freshness/hover/reset limits must be positive, sync skew nonnegative')
        if self.reset_lead > self.config.warmup_s:
            raise ValueError('Reset lead must fit within the 10 s warm-up')
        self.metrics = TrialMetrics(self.protocol, self.max_skew)
        self.odom = [deque(maxlen=2000), deque(maxlen=2000)]
        self.fcu = [None, None]
        self.ready = [(False, 0.), (False, 0.)]
        self.reset_results = [None, None]
        self.reset_wall_start = None
        self.summary_written = False
        self.last_tick = None
        self.output = Path(rospy.get_param('~output_dir', 'trial_' + time.strftime('%Y%m%d_%H%M%S')))
        if self.output.exists():
            raise FileExistsError('Refusing existing trial output directory: ' + str(self.output))
        self.publishers = [rospy.Publisher('/' + name + '/position_cmd', PositionCommand, queue_size=1) for name in self.names]
        self.subscribers = []
        for i, name in enumerate(self.names):
            self.subscribers.extend([
                rospy.Subscriber('/' + name + '/mavros/local_position/odom', Odometry, self._odom_cb, callback_args=i, queue_size=100),
                rospy.Subscriber('/' + name + '/mavros/state', State, self._state_cb, callback_args=i, queue_size=10),
                rospy.Subscriber('/' + name + '/px4ctrl/controller_ready', Bool, self._ready_cb, callback_args=i, queue_size=10)])
        self.start_service = rospy.Service('~start_trial', Trigger, self._start)
        self.timer = rospy.Timer(rospy.Duration(1./self.config.rate), self._tick)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo('Waiting for explicit ~start_trial after both controllers report valid OFFBOARD hover. No arming/takeoff is issued.')

    def _odom_cb(self, msg, i):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        sample = OdomSample(msg.header.stamp.to_sec(), (p.x, p.y, p.z), (v.x, v.y, v.z), msg.header.frame_id)
        if sample.stamp <= 0 or not all(map(math.isfinite, sample.position + sample.velocity)):
            return
        with self.lock:
            if self.odom[i] and sample.stamp <= self.odom[i][-1].stamp:
                return
            self.odom[i].append(sample)

    def _state_cb(self, msg, i):
        with self.lock:
            self.fcu[i] = (msg.header.stamp.to_sec(), msg.connected, msg.armed, msg.mode)

    def _ready_cb(self, msg, i):
        with self.lock:
            self.ready[i] = (msg.data, self.rospy.Time.now().to_sec())

    def _valid(self, now, require_hover=False):
        for i in range(2):
            if not self.odom[i]:
                return False, 'Missing odometry'
            sample, state = self.odom[i][-1], self.fcu[i]
            if sample.frame != self.frame or not 0 <= now - sample.stamp <= self.odom_timeout:
                return False, 'Odometry frame mismatch or stale/future odometry'
            if state is None or not 0 <= now - state[0] <= self.state_timeout or not state[1] or not state[2] or state[3] != 'OFFBOARD':
                return False, 'FCU must remain connected, armed and OFFBOARD with fresh state'
            if require_hover:
                ready, stamp = self.ready[i]
                if not ready or not 0 <= now - stamp <= self.ready_timeout:
                    return False, 'Controller has not acknowledged valid hover readiness'
                window = [s for s in self.odom[i] if s.stamp >= now - self.hover_window]
                if not window or window[0].stamp > now - self.hover_window + self.odom_timeout:
                    return False, 'Insufficient hover history'
                if any(math.sqrt(sum(v*v for v in s.velocity)) > self.hover_speed for s in window):
                    return False, 'Vehicle has not settled into hover'
        return True, ''

    def _start(self, _request):
        with self.lock:
            now = self.rospy.Time.now().to_sec()
            valid, reason = self._valid(now, require_hover=True)
            if self.protocol.state != 'idle' or not valid:
                return self.TriggerResponse(success=False, message=reason or 'Trial already started')
            try:
                self.output.mkdir(parents=True, exist_ok=False)
            except OSError as exc:
                return self.TriggerResponse(success=False, message=str(exc))
            self.protocol.start(now, [samples[-1].position for samples in self.odom])
            return self.TriggerResponse(success=True, message='Shared 10 s warm-up started; scoring awaits both reset acknowledgements')

    def _request_reset(self, index):
        service = '/' + self.names[index] + '/px4ctrl/reset_online'
        try:
            remaining = self.protocol.reset_epoch - self.rospy.Time.now().to_sec()
            if remaining <= 0:
                raise RuntimeError('Reset epoch already elapsed')
            self.rospy.wait_for_service(service, timeout=min(self.reset_timeout, remaining))
            with self.lock:
                if self.protocol.state != 'resetting' or self.rospy.Time.now().to_sec() >= self.protocol.reset_epoch:
                    raise RuntimeError('Reset reservation was aborted or missed the common epoch')
            response = self.rospy.ServiceProxy(service, self.Trigger)()
            # Record receipt time before waiting for the timer's lock: an
            # acknowledgement received before the epoch remains timely even
            # when the next 100 Hz callback runs just after that epoch.
            result = (bool(response.success), response.message, self.rospy.Time.now().to_sec())
        except Exception as exc:
            result = (False, str(exc), self.rospy.Time.now().to_sec())
        with self.lock:
            self.reset_results[index] = result

    def _reserve_resets(self):
        # XML-RPC parameter calls and service waits run outside the 100 Hz timer.
        try:
            for name in self.names:
                self.rospy.set_param('/' + name + '/px4ctrl/reset_epoch', self.protocol.reset_epoch)
            with self.lock:
                if self.protocol.state != 'resetting' or self.rospy.Time.now().to_sec() >= self.protocol.reset_epoch:
                    return
                for i in range(2):
                    threading.Thread(target=self._request_reset, args=(i,), daemon=True).start()
        except Exception as exc:
            with self.lock:
                self.reset_results = [(False, str(exc), self.rospy.Time.now().to_sec())] * 2

    def _publish(self, now, references):
        stamp = self.rospy.Time.from_sec(now)
        for publisher, reference in zip(self.publishers, references):
            command = self.PositionCommand()
            command.header.stamp, command.header.frame_id = stamp, self.frame
            command.trajectory_id = 1
            if hasattr(command, 'trajectory_flag'):
                command.trajectory_flag = getattr(command, 'TRAJECTORY_STATUS_READY', 1)
            for field, vector in zip(('position', 'velocity', 'acceleration', 'jerk'), reference):
                target = getattr(command, field)
                target.x, target.y, target.z = vector
            command.yaw, command.yaw_dot = self.config.yaw, 0.0
            publisher.publish(command)

    def _collect_metrics(self, now):
        # Use unique stamped samples, nearest-neighbor pairing, never current-time
        # relabelling of a held position. Delay slightly to await the other stream.
        if self.protocol.scoring_start is None or not all(self.odom):
            return
        first_candidates = [s for s in self.odom[0] if self.metrics.last_stamps[0] < s.stamp <= now - self.max_skew]
        second_candidates = [s for s in self.odom[1] if s.stamp > self.metrics.last_stamps[1]]
        second_stamps = [s.stamp for s in second_candidates]
        for first in first_candidates:
            if now - first.stamp > self.odom_timeout or first.frame != self.frame:
                continue
            index = bisect.bisect_left(second_stamps, first.stamp)
            choices = [second_candidates[j] for j in (index-1, index)
                       if 0 <= j < len(second_candidates) and second_candidates[j].stamp > self.metrics.last_stamps[1]]
            if choices:
                second = min(choices, key=lambda s: abs(s.stamp-first.stamp))
                if 0 <= now - second.stamp <= self.odom_timeout and second.frame == self.frame:
                    self.metrics.add(first, second)

    def _save_summary(self, now):
        if self.summary_written or not self.output.is_dir():
            return
        summary = self.metrics.report(now)
        summary['trajectory_parameters'] = self.config.__dict__
        summary['scheduled_reset_epoch'] = self.protocol.reset_epoch
        summary['reset_reservations'] = self.reset_results
        summary['measurement_acceptance'] = {'odom_timeout_s': self.odom_timeout, 'max_sync_skew_s': self.max_skew}
        (self.output / 'summary.json').write_text(json.dumps(summary, indent=2))
        with (self.output / 'paired_metrics.jsonl').open('w') as stream:
            for bin_id, record in sorted(self.metrics.records.items()):
                stream.write(json.dumps({'bin': bin_id, **record}) + '\n')
        self.summary_written = True
        self.rospy.loginfo('Trial summary written: %s (coverage %.3f)', self.output, summary['coverage'])

    def _tick(self, _event):
        with self.lock:
            now = self.rospy.Time.now().to_sec()
            if self.protocol.state == 'idle':
                return
            if self.last_tick is not None and now < self.last_tick:
                self.protocol.abort('ROS clock moved backwards')
            self.last_tick = now
            self._collect_metrics(now)
            valid, reason = self._valid(now)
            if not valid and self.protocol.state != 'aborted':
                self.protocol.abort(reason)
                self.rospy.logerr('Stopping both position-command streams: %s', reason)
            if self.protocol.scoring_start is not None:
                end = self.protocol.scoring_start + self.config.scoring_s
                if now >= end + self.odom_timeout:
                    self._save_summary(now)
            if self.protocol.state == 'aborted':
                # Keep observing scoring interval (backup counts), never restart commands.
                if self.protocol.scoring_start is None:
                    self._save_summary(now)
                return
            if self.protocol.reset_due(now, self.reset_lead):
                epoch = self.protocol.start_time + self.config.warmup_s
                if now >= epoch:
                    self.protocol.abort('Missed reset reservation before the fixed warm-up end')
                    return
                self.protocol.begin_reset(now, epoch, self.reset_lead)
                self.reset_wall_start = time.monotonic()
                threading.Thread(target=self._reserve_resets, daemon=True).start()
            if self.protocol.state == 'resetting':
                if any(r is not None and not r[0] for r in self.reset_results):
                    self.protocol.abort('A controller reset failed: ' + repr(self.reset_results))
                    return
                if time.monotonic() - self.reset_wall_start > self.reset_timeout:
                    self.protocol.abort('Controller reset reservation exceeded its wall-time deadline')
                    return
                if all(r is not None for r in self.reset_results):
                    ack_time = max(r[2] for r in self.reset_results)
                    if not self.protocol.acknowledge_resets(ack_time, [r[0] for r in self.reset_results]):
                        return
                    self.rospy.loginfo('Both reset reservations accepted for epoch %.6f', self.protocol.reset_epoch)
                elif now >= self.protocol.reset_epoch:
                    self.protocol.abort('Controller reset reservation missed the shared epoch')
                    return
            if self.protocol.activate_scoring(now):
                self.rospy.loginfo('Shared 60 s scoring started at reserved epoch %.6f', self.protocol.scoring_start)
            if self.protocol.scoring_start is not None:
                stopping_start = self.protocol.scoring_start + self.config.scoring_s
                if now >= stopping_start:
                    self.protocol.state = 'holding' if now >= stopping_start + self.config.stopping_s else 'stopping'
            self._publish(now, self.protocol.reference(now))

    def _shutdown(self):
        with self.lock:
            self._save_summary(self.rospy.Time.now().to_sec())


def main():
    import rospy
    rospy.init_node('dual_figure8_commander')
    DualDroneManager()
    rospy.spin()


if __name__ == '__main__':
    main()
