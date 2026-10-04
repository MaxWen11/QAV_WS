#!/usr/bin/env python3
"""Export synchronized flight records for offline prior training (Section VI-B).

Reads, for one vehicle namespace, the recorded streams

    <ns>/mavros/imu/data                 IMU specific force and attitude
    <ns>/mavros/local_position/odom      position, velocity and attitude (EKF2)
    <ns>/mavros/setpoint_raw/attitude    executed attitude and normalized thrust

and aligns IMU/odometry by nearest timestamps. Commands use the latest
setpoint at or before both measurement timestamps, with a maximum age.
For each odometry sample the record contains

    state  x = [p, v]                    inertial position and velocity
    a      R_odom f_imu - g e3           IMU acceleration rotated into the inertial
                                         frame with the odometry attitude, plus gravity
    u      (F_T,cmd/m0) R_cmd e3 - g e3  executed net acceleration command, from the
                                         latest attitude setpoint at or before the
                                         measurement, with m0 = 0.9 kg, g = 9.81 m/s^2

so each measured response remains paired with the input executed when it was
measured. F_T,cmd follows the normalized-thrust mapping of the controller
(hover thrust -> m0 g). R_cmd is expressed in the odometry frame: px4ctrl
publishes q_sp = q_imu q_odom^-1 q_cmd, so q_cmd = q_odom q_imu^-1 q_sp.

Each bag is one flight and carries its physical environment label: both fans
off, only fan A on, or both fans on. Rows are partitioned chronologically
within each environment into approximately 70/15/15 percent blocks; adjacent
blocks in the same flight have a 2 s exclusion interval. The fixed split
column is consumed by training and normalization. These records are state
contexts, not measurements of subsequently requested policy-action probes.

    python3 utils/export_flight_records.py --namespace /drone1 \\
        --flight flight_01.bag:fans_off --flight flight_02.bag:fan_a \\
        --flight flight_03.bag:fans_on --output data/offline_records.csv
"""
import argparse
import bisect
import csv
import math
from pathlib import Path

ENVIRONMENTS = ('fans_off', 'fan_a', 'fans_on')
COLUMNS = ['timestamp', 'flight', 'environment', 'p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z',
           'u_x', 'u_y', 'u_z', 'a_x', 'a_y', 'a_z', 'split']
MASS = 0.9
GRAVITY = 9.81


def quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quaternion_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Degenerate quaternion')
    return tuple(value / norm for value in q)


def quaternion_inverse(q):
    w, x, y, z = quaternion_normalize(q)
    return (w, -x, -y, -z)


def rotate(q, vector):
    w, x, y, z = quaternion_normalize(q)
    rotated = quaternion_multiply(quaternion_multiply((w, x, y, z), (0.0, *vector)), (w, -x, -y, -z))
    return rotated[1:]


def net_acceleration_label(q_odom, specific_force, gravity=GRAVITY):
    """Measured net inertial acceleration: R_odom f_imu - g e3."""
    world = rotate(q_odom, specific_force)
    return (world[0], world[1], world[2] - gravity)


def executed_net_acceleration(q_setpoint, thrust, q_odom, q_imu, hover_thrust,
                              mass=MASS, gravity=GRAVITY):
    """u = (F_T,cmd/m0) R_cmd e3 - g e3 with F_T,cmd = m0 g thrust / hover_thrust."""
    if not 0.0 < hover_thrust <= 1.0 or not 0.0 <= thrust <= 1.0:
        raise ValueError('Normalized thrust and hover thrust must lie in [0, 1]')
    if not math.isfinite(mass) or not math.isfinite(gravity) or mass <= 0 or gravity <= 0:
        raise ValueError('Mass and gravity must be finite and positive')
    q_cmd = quaternion_multiply(quaternion_multiply(q_odom, quaternion_inverse(q_imu)), q_setpoint)
    zb = rotate(q_cmd, (0.0, 0.0, 1.0))
    force_per_mass = (mass * gravity * thrust / hover_thrust) / mass
    return (force_per_mass * zb[0], force_per_mass * zb[1], force_per_mass * zb[2] - gravity)


def nearest(stamps, stamp):
    """Index of the nearest stamp in a sorted list."""
    index = bisect.bisect_left(stamps, stamp)
    candidates = [j for j in (index - 1, index) if 0 <= j < len(stamps)]
    return min(candidates, key=lambda j: abs(stamps[j] - stamp)) if candidates else None


def latest_at_or_before(stamps, stamp):
    index = bisect.bisect_right(stamps, stamp) - 1
    return index if index >= 0 else None


def synchronize(odometry, imu, setpoints, flight, environment, hover_thrust,
                max_skew=0.01, command_max_age=0.03, mass=MASS, gravity=GRAVITY):
    """Nearest-neighbor synchronization of the three streams.

    odometry:  [(stamp, position, velocity_world, q_odom)]
    imu:       [(stamp, specific_force_body, q_imu)]
    setpoints: [(stamp, q_setpoint, thrust)]
    All lists are sorted by stamp; quaternions are (w, x, y, z).
    """
    if environment not in ENVIRONMENTS:
        raise ValueError('environment must be one of ' + ', '.join(ENVIRONMENTS))
    if not math.isfinite(max_skew) or not math.isfinite(command_max_age) or max_skew < 0 or command_max_age <= 0:
        raise ValueError('Synchronization tolerances must be finite and nonnegative; command age must be positive')
    imu_stamps = [sample[0] for sample in imu]
    odom_stamps = [sample[0] for sample in odometry]
    setpoint_stamps = [sample[0] for sample in setpoints]
    for stamp, position, velocity, q_odom in odometry:
        i = nearest(imu_stamps, stamp)
        if i is None or abs(imu_stamps[i] - stamp) > max_skew:
            continue
        c = latest_at_or_before(setpoint_stamps, min(stamp, imu_stamps[i]))
        # The same command must cover the aligned state and acceleration;
        # a setpoint transition between their timestamps is ambiguous.
        if c is None or c != latest_at_or_before(setpoint_stamps, max(stamp, imu_stamps[i])):
            continue
        if max(stamp, imu_stamps[i]) - setpoint_stamps[c] > command_max_age:
            continue
        command_stamp, q_setpoint, thrust = setpoints[c]
        # Orientations at the setpoint time undo the published frame mapping.
        o = nearest(odom_stamps, command_stamp)
        j = nearest(imu_stamps, command_stamp)
        if o is None or j is None or abs(odom_stamps[o] - command_stamp) > max_skew or \
                abs(imu_stamps[j] - command_stamp) > max_skew:
            continue
        u = executed_net_acceleration(q_setpoint, thrust, odometry[o][3], imu[j][2], hover_thrust,
                                      mass, gravity)
        a = net_acceleration_label(q_odom, imu[i][1], gravity)
        row = [stamp, flight, environment, *position, *velocity, *u, *a]
        if all(math.isfinite(float(value)) for value in [row[0]] + row[3:]):
            yield row


def partition_records(rows, boundary_gap=2.0):
    """Freeze chronological 70/15/15 blocks with symmetric boundary exclusion.

    A cutoff is halfway between the two adjoining sample timestamps; remove
    the 1 s interval on each side when both samples belong to one flight.
    Boundaries between separate flights require no within-flight exclusion.
    """
    if not math.isfinite(boundary_gap) or boundary_gap < 0:
        raise ValueError('boundary_gap must be finite and nonnegative')
    selected = []
    for environment in ENVIRONMENTS:
        members = sorted((list(row) for row in rows if row[2] == environment), key=lambda row: (row[0], row[1]))
        if not members:
            continue
        n = len(members)
        first, second = int(round(.70*n)), int(round(.85*n))
        if not 0 < first < second < n:
            raise ValueError('Each included environment needs enough records for three chronological blocks')
        exclusions = []
        for boundary in (first, second):
            before, after = members[boundary-1], members[boundary]
            if before[1] == after[1]:
                exclusions.append((before[1], .5*(before[0]+after[0])))
        retained = {'train': 0, 'validation': 0, 'test': 0}
        for index, row in enumerate(members):
            if any(row[1] == flight and abs(row[0]-cutoff) < boundary_gap/2
                   for flight, cutoff in exclusions):
                continue
            split = 'train' if index < first else 'validation' if index < second else 'test'
            retained[split] += 1
            selected.append(row+[split])
        if not all(retained.values()):
            raise ValueError('The 2 s boundary exclusions leave an empty block; collect longer flights')
    return sorted(selected, key=lambda row: (row[0], row[1]))


def write_records(rows, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        writer.writerows(rows)


def read_bag(bag_path, namespace):
    """Read the three MAVROS streams of one flight (ROS1 rosbag API)."""
    import rosbag  # Available after sourcing the ROS workspace.
    prefix = namespace.rstrip('/')
    topics = {prefix + '/mavros/imu/data': 'imu',
              prefix + '/mavros/local_position/odom': 'odom',
              prefix + '/mavros/setpoint_raw/attitude': 'setpoint'}
    odometry, imu, setpoints = [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=list(topics)):
            stamp = message.header.stamp.to_sec()
            kind = topics[topic]
            if kind == 'odom':
                pose, twist = message.pose.pose, message.twist.twist
                q = (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                # MAVROS publishes the odometry twist in the child (body) frame.
                velocity = rotate(q, (twist.linear.x, twist.linear.y, twist.linear.z))
                odometry.append((stamp, (pose.position.x, pose.position.y, pose.position.z), velocity, q))
            elif kind == 'imu':
                q = (message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z)
                force = (message.linear_acceleration.x, message.linear_acceleration.y,
                         message.linear_acceleration.z)
                imu.append((stamp, force, q))
            else:
                q = (message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z)
                setpoints.append((stamp, q, message.thrust))
    for stream in (odometry, imu, setpoints):
        stream.sort(key=lambda sample: sample[0])
    return odometry, imu, setpoints


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--flight', action='append', required=True,
                        help='bag:environment with environment in ' + ', '.join(ENVIRONMENTS))
    parser.add_argument('--namespace', default='/drone1')
    parser.add_argument('--output', required=True, help='CSV file to write')
    parser.add_argument('--hover-thrust', type=float, default=0.5,
                        help='normalized thrust at hover (thrust_model/hover_percentage)')
    parser.add_argument('--mass', type=float, default=MASS, help='nominal mass m0, kg')
    parser.add_argument('--gravity', type=float, default=GRAVITY)
    parser.add_argument('--max-skew', type=float, default=0.01, help='largest IMU/odometry stamp offset, s')
    parser.add_argument('--command-max-age', type=float, default=0.03,
                        help='oldest executed setpoint paired with a measurement, s')
    args = parser.parse_args(argv)
    rows = []
    for item in args.flight:
        bag_path, _, environment = item.rpartition(':')
        if not bag_path or environment not in ENVIRONMENTS:
            parser.error('--flight expects bag:environment with environment in ' + ', '.join(ENVIRONMENTS))
        odometry, imu, setpoints = read_bag(bag_path, args.namespace)
        count = len(rows)
        rows.extend(synchronize(odometry, imu, setpoints, Path(bag_path).stem, environment,
                                args.hover_thrust, args.max_skew, args.command_max_age,
                                args.mass, args.gravity))
        print(f'{bag_path} ({environment}): {len(rows) - count} records')
    try:
        rows = partition_records(rows)
    except ValueError as exc:
        parser.error(str(exc))
    write_records(rows, args.output)
    print(f'Wrote synchronized records with fixed split to {args.output}')


if __name__ == '__main__':
    main()
