#!/usr/bin/env python3
"""Export synchronized flight records for offline prior training.

Reads the px4ctrl debug topic (``/<drone>/debugPx4ctrl``, 100 Hz) from one or
more rosbags and writes the CSV consumed by ``utils/offline_GANs_Train.py``.
Record k pairs the state, executed net acceleration and virtual input eta of
control cycle k with the measured net acceleration reported at cycle k+1,
i.e. the response to the command executed over that control interval.

    python3 utils/export_flight_records.py flight_01.bag flight_02.bag \\
        --topic /drone1/debugPx4ctrl --output data/offline_records.csv
"""
import argparse
import csv
import math
from pathlib import Path

COLUMNS = ['timestamp', 'p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z',
           'u_x', 'u_y', 'u_z', 'a_x', 'a_y', 'a_z', 'eta_x', 'eta_y', 'eta_z']


def records_from_messages(messages, max_gap=0.015):
    """Pair consecutive debug messages; ``messages`` holds (stamp, msg) in time order."""
    previous = None
    for stamp, message in messages:
        if previous is not None:
            last_stamp, last = previous
            if 0.0 < stamp - last_stamp <= max_gap and last.controller_valid and message.controller_valid:
                row = [last_stamp, last.real_x, last.real_y, last.real_z,
                       last.real_vx, last.real_vy, last.real_vz,
                       last.des_a_x, last.des_a_y, last.des_a_z,
                       message.fb_a_x, message.fb_a_y, message.fb_a_z,
                       last.eta[0], last.eta[1], last.eta[2]]
                if all(math.isfinite(value) for value in row):
                    yield row
        previous = (stamp, message)


def write_records(rows, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bags', nargs='+', help='rosbag files recorded during flight')
    parser.add_argument('--topic', default='/drone1/debugPx4ctrl')
    parser.add_argument('--output', required=True, help='CSV file to write')
    parser.add_argument('--max-gap', type=float, default=0.015, help='largest cycle spacing paired, s')
    args = parser.parse_args(argv)
    import rosbag  # ROS1 Python API, available after sourcing the workspace.
    rows = []
    for bag_path in args.bags:
        with rosbag.Bag(bag_path) as bag:
            messages = sorted(((message.header.stamp.to_sec(), message)
                               for _, message, _ in bag.read_messages(topics=[args.topic])),
                              key=lambda item: item[0])
        count = len(rows)
        rows.extend(records_from_messages(messages, args.max_gap))
        print(f'{bag_path}: {len(rows) - count} records')
    rows.sort(key=lambda row: row[0])
    write_records(rows, args.output)
    print(f'Wrote {len(rows)} records to {args.output}')


if __name__ == '__main__':
    main()
