#!/usr/bin/env python3
"""Interactive CSV collector for /sensor_distances and Odin pose."""

import argparse
import csv
import math
import os
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SensorOdinTopicCollector(Node):
    """Cache latest distance-sensor topic data and Odin pose."""

    def __init__(
        self,
        sensor_topic: str,
        pose_topic: str,
        sensor_count: int,
    ) -> None:
        super().__init__('sensor_odin_topic_collector')
        self.sensor_count = sensor_count
        self._lock = threading.Lock()
        self._latest_distances = [math.nan] * sensor_count
        self._latest_sensor_receive_time_s = math.nan
        self._latest_pose: Optional[PoseStamped] = None
        self._latest_pose_receive_time_s = math.nan

        self.sensor_sub = self.create_subscription(
            Float32MultiArray,
            sensor_topic,
            self._sensor_callback,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self._pose_callback,
            10,
        )
        self.get_logger().info(
            'Collecting %d sensors from %s; Odin pose topic: %s'
            % (sensor_count, sensor_topic, pose_topic)
        )

    def _sensor_callback(self, msg: Float32MultiArray) -> None:
        distances = [math.nan] * self.sensor_count
        for index in range(min(self.sensor_count, len(msg.data))):
            distances[index] = float(msg.data[index])
        with self._lock:
            self._latest_distances = distances
            self._latest_sensor_receive_time_s = self._now_sec()

    def _pose_callback(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg
            self._latest_pose_receive_time_s = self._now_sec()

    def snapshot(self) -> dict:
        now_s = self._now_sec()
        with self._lock:
            distances = list(self._latest_distances)
            sensor_receive_time_s = self._latest_sensor_receive_time_s
            pose = self._latest_pose
            pose_receive_time_s = self._latest_pose_receive_time_s

        row = {
            'ros_time_s': now_s,
            'unix_time_s': time.time(),
            'sensor_age_s': (
                now_s - sensor_receive_time_s
                if math.isfinite(sensor_receive_time_s)
                else math.nan
            ),
        }
        for index, distance in enumerate(distances):
            row['sensor_%d_mm' % index] = distance

        if pose is None:
            row.update(
                {
                    'odin_frame_id': '',
                    'odin_stamp_s': math.nan,
                    'odin_x_m': math.nan,
                    'odin_y_m': math.nan,
                    'odin_z_m': math.nan,
                    'odin_qx': math.nan,
                    'odin_qy': math.nan,
                    'odin_qz': math.nan,
                    'odin_qw': math.nan,
                    'odin_yaw_rad': math.nan,
                    'odin_pose_age_s': math.nan,
                }
            )
            return row

        position = pose.pose.position
        orientation = pose.pose.orientation
        row.update(
            {
                'odin_frame_id': pose.header.frame_id,
                'odin_stamp_s': _stamp_to_sec(pose.header.stamp),
                'odin_x_m': position.x,
                'odin_y_m': position.y,
                'odin_z_m': position.z,
                'odin_qx': orientation.x,
                'odin_qy': orientation.y,
                'odin_qz': orientation.z,
                'odin_qw': orientation.w,
                'odin_yaw_rad': _yaw_from_quaternion(orientation),
                'odin_pose_age_s': now_s - pose_receive_time_s,
            }
        )
        return row

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _build_fieldnames(sensor_count: int) -> list[str]:
    fields = ['sample_index', 'ros_time_s', 'unix_time_s', 'sensor_age_s']
    fields.extend('sensor_%d_mm' % index for index in range(sensor_count))
    fields.extend(
        [
            'odin_frame_id',
            'odin_stamp_s',
            'odin_x_m',
            'odin_y_m',
            'odin_z_m',
            'odin_qx',
            'odin_qy',
            'odin_qz',
            'odin_qw',
            'odin_yaw_rad',
            'odin_pose_age_s',
        ]
    )
    return fields


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='交互式采集 /sensor_distances 和 Odin 定位到 CSV。'
    )
    parser.add_argument(
        '-o', '--output',
        default='sensor_odin_samples.csv',
        help='CSV 输出文件路径，默认写到当前目录的 sensor_odin_samples.csv',
    )
    parser.add_argument(
        '--sensor-topic',
        default='/sensor_distances',
        help='Float32MultiArray 测距话题，默认 /sensor_distances。',
    )
    parser.add_argument(
        '--sensor-count',
        type=int,
        default=8,
        help='写入 CSV 的传感器数量，默认 8。',
    )
    parser.add_argument(
        '--pose-topic',
        default='/odin1/relocation',
        help='Odin PoseStamped 话题，默认 /odin1/relocation。',
    )
    return parser.parse_args()


def main(args=None) -> None:
    cli_args = _parse_args()
    if cli_args.sensor_count <= 0:
        raise SystemExit('--sensor-count must be positive')

    rclpy.init(args=args)
    node = SensorOdinTopicCollector(
        sensor_topic=cli_args.sensor_topic,
        pose_topic=cli_args.pose_topic,
        sensor_count=cli_args.sensor_count,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fieldnames = _build_fieldnames(cli_args.sensor_count)
    file_exists = os.path.exists(cli_args.output)
    sample_index = 0

    try:
        with open(cli_args.output, 'a', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            if not file_exists or os.path.getsize(cli_args.output) == 0:
                writer.writeheader()
                csv_file.flush()

            print('输出文件: %s' % os.path.abspath(cli_args.output))
            print('输入 y 采集一组，输入 q 退出。')
            while rclpy.ok():
                answer = input('是否采集当前数据？[y/q] ').strip().lower()
                if answer in ('q', 'quit', 'exit'):
                    break
                if answer != 'y':
                    continue

                sample_index += 1
                row = node.snapshot()
                row['sample_index'] = sample_index
                writer.writerow(row)
                csv_file.flush()
                print(
                    '已记录第 %d 组：sensor=%s, odin=(%.3f, %.3f, yaw=%.3f)'
                    % (
                        sample_index,
                        [
                            row['sensor_%d_mm' % index]
                            for index in range(cli_args.sensor_count)
                        ],
                        row['odin_x_m'],
                        row['odin_y_m'],
                        row['odin_yaw_rad'],
                    )
                )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
