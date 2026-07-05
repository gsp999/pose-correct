#!/usr/bin/env python3
"""Interactive CSV collector for distance sensors and Odin pose."""

import argparse
import csv
import math
import os
import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

try:
    import serial
except ImportError:  # pragma: no cover - depends on target robot image
    serial = None


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SensorOdinCollector(Node):
    """Poll serial distance sensors and cache the latest Odin pose."""

    def __init__(
        self,
        port_names: List[str],
        baudrate: int,
        pose_topic: str,
        timer_period_s: float,
    ) -> None:
        super().__init__('sensor_odin_data_collector')
        if serial is None:
            raise RuntimeError(
                'pyserial is required. Install python3-serial or pyserial.'
            )

        self.port_names = port_names
        self.baudrate = baudrate
        self.serials: List[Optional[serial.Serial]] = []
        self.buffers = [bytearray() for _ in self.port_names]
        self.last_distances = [math.nan] * len(self.port_names)
        self.miss_counts = [0] * len(self.port_names)
        self.last_reconnect_attempt = [0.0] * len(self.port_names)
        self.reconnect_interval_s = 1.0

        self._lock = threading.Lock()
        self._latest_pose: Optional[PoseStamped] = None
        self._latest_pose_receive_time_s = math.nan

        for index in range(len(self.port_names)):
            self.serials.append(None)
            self._try_open_serial(index, force=True)

        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._pose_callback, 10
        )
        self.timer = self.create_timer(timer_period_s, self._timer_callback)
        self.get_logger().info(
            'Collecting %d sensors from %s; Odin pose topic: %s'
            % (len(self.port_names), ', '.join(self.port_names), pose_topic)
        )

    def _try_open_serial(self, index: int, force: bool = False) -> None:
        now = time.monotonic()
        if (
            not force
            and now - self.last_reconnect_attempt[index]
            < self.reconnect_interval_s
        ):
            return

        self.last_reconnect_attempt[index] = now
        port = self.port_names[index]
        try:
            self.serials[index] = serial.Serial(
                port, self.baudrate, timeout=0
            )
            self.buffers[index].clear()
            self.miss_counts[index] = 0
            self.get_logger().info('成功打开串口: %s' % port)
        except serial.SerialException:
            self.serials[index] = None

    @staticmethod
    def parse_packet(data: bytes) -> float:
        """Parse one 195-byte packet and return a confident distance in mm."""
        for offset in range(10, 195, 15):
            if offset + 15 <= len(data):
                confidence = data[offset + 8]
                if confidence == 100:
                    distance = data[offset] | (data[offset + 1] << 8)
                    return float(distance)
        return math.nan

    def _pose_callback(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg
            self._latest_pose_receive_time_s = self._now_sec()

    def _timer_callback(self) -> None:
        for index in range(len(self.port_names)):
            ser = self.serials[index]
            if ser is not None and ser.is_open:
                try:
                    if ser.in_waiting > 0:
                        self.buffers[index].extend(ser.read(ser.in_waiting))

                    while len(self.buffers[index]) >= 195:
                        if self.buffers[index][0] == 0xAA:
                            packet = self.buffers[index][:195]
                            distance = self.parse_packet(packet)
                            if math.isnan(distance):
                                self.miss_counts[index] += 1
                            else:
                                self.miss_counts[index] = 0
                                with self._lock:
                                    self.last_distances[index] = distance
                            self.buffers[index] = self.buffers[index][195:]
                        else:
                            self.buffers[index].pop(0)

                        if self.miss_counts[index] >= 5:
                            with self._lock:
                                had_value = not math.isnan(
                                    self.last_distances[index]
                                )
                                self.last_distances[index] = math.nan
                            if had_value:
                                self.get_logger().warn(
                                    '串口 %s 掉线，连续 5 次未读取到有效数据。'
                                    % self.port_names[index]
                                )
                except Exception as exc:
                    self.get_logger().error(
                        '读取串口 %s 失败，设备可能已拔出: %s'
                        % (self.port_names[index], exc)
                    )
                    if ser is not None:
                        ser.close()
                    self.serials[index] = None
                    self.buffers[index].clear()
                    self.miss_counts[index] = 0
                    with self._lock:
                        self.last_distances[index] = math.nan
            else:
                with self._lock:
                    self.last_distances[index] = math.nan
                self._try_open_serial(index)

    def snapshot(self) -> dict:
        now_s = self._now_sec()
        with self._lock:
            distances = list(self.last_distances)
            pose = self._latest_pose
            pose_receive_time_s = self._latest_pose_receive_time_s

        row = {
            'ros_time_s': now_s,
            'unix_time_s': time.time(),
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

    def close_serials(self) -> None:
        for ser in self.serials:
            if ser is not None and ser.is_open:
                ser.close()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _build_fieldnames(sensor_count: int) -> List[str]:
    fields = ['sample_index', 'ros_time_s', 'unix_time_s']
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
        description='交互式采集七路测距传感器和 Odin 定位到 CSV。'
    )
    parser.add_argument(
        '-o', '--output',
        default='sensor_odin_samples.csv',
        help='CSV 输出文件路径，默认写到当前目录的 sensor_odin_samples.csv',
    )
    parser.add_argument(
        '--sensor-count',
        type=int,
        default=7,
        help='采集传感器数量，默认 7。',
    )
    parser.add_argument(
        '--port-prefix',
        default='/dev/ttyCH9344USB',
        help='串口名前缀，默认 /dev/ttyCH9344USB。',
    )
    parser.add_argument(
        '--port-names',
        default='',
        help='逗号分隔的串口完整路径；设置后覆盖 --sensor-count/--port-prefix。',
    )
    parser.add_argument(
        '--baudrate',
        type=int,
        default=230400,
        help='串口波特率，默认 230400。',
    )
    parser.add_argument(
        '--pose-topic',
        default='/odin1/relocation',
        help='Odin PoseStamped 话题，默认 /odin1/relocation。',
    )
    parser.add_argument(
        '--timer-period',
        type=float,
        default=0.01,
        help='串口轮询周期秒，默认 0.01。',
    )
    return parser.parse_args()


def _port_names(args: argparse.Namespace) -> List[str]:
    if args.port_names:
        return [
            item.strip()
            for item in args.port_names.split(',')
            if item.strip()
        ]
    return [
        '%s%d' % (args.port_prefix, index)
        for index in range(args.sensor_count)
    ]


def main(args=None) -> None:
    cli_args = _parse_args()
    port_names = _port_names(cli_args)
    if not port_names:
        raise SystemExit('至少需要一个传感器串口。')

    rclpy.init(args=args)
    node = SensorOdinCollector(
        port_names=port_names,
        baudrate=cli_args.baudrate,
        pose_topic=cli_args.pose_topic,
        timer_period_s=cli_args.timer_period,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fieldnames = _build_fieldnames(len(port_names))
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
                            for index in range(len(port_names))
                        ],
                        row['odin_x_m'],
                        row['odin_y_m'],
                        row['odin_yaw_rad'],
                    )
                )
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serials()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
