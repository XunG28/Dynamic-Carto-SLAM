#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""from rosbag2 to TUM"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def main() -> None:
    p = argparse.ArgumentParser(
        description='Export PoseStamped topic from a rosbag2 folder to TUM (timestamp tx ty tz qx qy qz qw).'
    )
    p.add_argument('bag_dir', type=Path, help='rosbag2 file（ *.db3）')
    p.add_argument('topic', type=str, help=' /tracked_pose or /eval/ground_truth/pose')
    p.add_argument('-o', '--output', type=Path, required=True, help='output .tum file')
    args = p.parse_args()

    storage_options = StorageOptions(uri=str(args.bag_dir), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in type_map:
        print(f'error: topic {args.topic} not found in bag', file=sys.stderr)
        print('available topics:', ', '.join(sorted(type_map)), file=sys.stderr)
        sys.exit(1)
    msg_cls = get_message(type_map[args.topic])

    rows: list[tuple[float, float, float, float, float, float, float, float]] = []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, msg_cls)
        ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        pos = msg.pose.position
        q = msg.pose.orientation
        rows.append((ts, pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w))

    rows.sort(key=lambda r: r[0])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as f:
        for r in rows:
            f.write(' '.join(f'{x:.9f}' for x in r) + '\n')
    print(f'write {len(rows)} rows -> {args.output}')


if __name__ == '__main__':
    main()
