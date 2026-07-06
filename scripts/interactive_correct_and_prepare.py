#!/usr/bin/env python3
"""Interactively correct gripper pose and trigger prepare translation."""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from r2_interfaces.srv import ToolAction
except ImportError:
    from ares_tool_interfaces.srv import ToolAction

from correct_pose_sensor_3_5 import correct_pose_from_odin, load_geometry


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yes(text: str) -> bool:
    return text.strip().lower() in ('y', 'yes')


class InteractiveCorrectPrepare(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('interactive_correct_prepare')
        self.args = args
        self._lock = threading.Lock()
        self._latest_distances = [math.nan] * args.sensor_count
        self._latest_sensor_receive_time_s = math.nan
        self._latest_pose: PoseStamped | None = None
        self._latest_pose_receive_time_s = math.nan

        self.create_subscription(
            Float32MultiArray,
            args.sensor_topic,
            self._sensor_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            args.pose_topic,
            self._pose_callback,
            10,
        )
        self._tool_client = self.create_client(ToolAction, args.tool_service)

    def _sensor_callback(self, msg: Float32MultiArray) -> None:
        distances = [math.nan] * self.args.sensor_count
        for index in range(min(self.args.sensor_count, len(msg.data))):
            distances[index] = float(msg.data[index])
        with self._lock:
            self._latest_distances = distances
            self._latest_sensor_receive_time_s = self._now_s()

    def _pose_callback(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg
            self._latest_pose_receive_time_s = self._now_s()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def snapshot(self) -> dict[str, float] | None:
        now_s = self._now_s()
        with self._lock:
            distances = list(self._latest_distances)
            sensor_time = self._latest_sensor_receive_time_s
            pose = self._latest_pose
            pose_time = self._latest_pose_receive_time_s

        sensor_age_s = now_s - sensor_time if math.isfinite(sensor_time) else math.inf
        pose_age_s = now_s - pose_time if math.isfinite(pose_time) else math.inf
        if sensor_age_s > self.args.sensor_max_age_s:
            print(
                '传感器数据不可用或过旧: age=%.3fs, max=%.3fs'
                % (sensor_age_s, self.args.sensor_max_age_s)
            )
            return None
        if pose is None or pose_age_s > self.args.pose_max_age_s:
            print(
                'Odin 位姿不可用或过旧: age=%.3fs, max=%.3fs'
                % (pose_age_s, self.args.pose_max_age_s)
            )
            return None
        if (
            self.args.sensor_3_index < 0
            or self.args.sensor_5_index < 0
            or self.args.sensor_3_index >= len(distances)
            or self.args.sensor_5_index >= len(distances)
        ):
            print('传感器索引越界')
            return None

        sensor_3_mm = distances[self.args.sensor_3_index]
        sensor_5_mm = distances[self.args.sensor_5_index]
        if not math.isfinite(sensor_3_mm) or not math.isfinite(sensor_5_mm):
            print(
                '传感器 3/5 数据无效: sensor_3=%s sensor_5=%s'
                % (sensor_3_mm, sensor_5_mm)
            )
            return None

        position = pose.pose.position
        return {
            'sensor_3_mm': sensor_3_mm,
            'sensor_5_mm': sensor_5_mm,
            'sensor_age_s': sensor_age_s,
            'pose_age_s': pose_age_s,
            'odin_x_m': float(position.x),
            'odin_y_m': float(position.y),
            'odin_yaw_rad': _yaw_from_quaternion(pose.pose.orientation),
        }

    def wait_for_tool_service(self) -> bool:
        return self._tool_client.wait_for_service(
            timeout_sec=self.args.service_wait_timeout_s
        )

    def call_prepare(self, length_m: float) -> bool:
        req = ToolAction.Request()
        req.action = 'prepare'
        req.args = [float(length_m), 0.0, 0.0, 0.0]
        future = self._tool_client.call_async(req)
        deadline = time.monotonic() + self.args.prepare_timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            print('prepare 调用超时: %.3fs' % self.args.prepare_timeout_s)
            return False
        result = future.result()
        if result.success:
            print('prepare 完成: ret=%d msg=%s' % (result.ret, result.message))
            return True
        print('prepare 失败: ret=%d msg=%s' % (result.ret, result.message))
        return False


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='交互式纠错并触发夹爪 prepare 平移。'
    )
    parser.add_argument('--sensor-topic', default='/sensor_distances')
    parser.add_argument('--pose-topic', default='/odin1/relocation')
    parser.add_argument('--tool-service', default='/ares_tool_node/tool_action')
    parser.add_argument('--sensor-count', type=int, default=8)
    parser.add_argument('--sensor-3-index', type=int, default=3)
    parser.add_argument('--sensor-5-index', type=int, default=5)
    parser.add_argument('--sensor-max-age-s', type=float, default=0.5)
    parser.add_argument('--pose-max-age-s', type=float, default=0.5)
    parser.add_argument('--service-wait-timeout-s', type=float, default=3.0)
    parser.add_argument('--prepare-timeout-s', type=float, default=20.0)
    parser.add_argument(
        '--base-length-m',
        type=float,
        default=0.3,
        help='Current/default prepare length in meters. Command = base + move.',
    )
    parser.add_argument(
        '--min-length-m',
        type=float,
        default=0.0,
        help='Minimum allowed prepare length in meters.',
    )
    parser.add_argument(
        '--max-length-m',
        type=float,
        default=0.5,
        help='Maximum allowed prepare length in meters.',
    )
    parser.add_argument('--deadband-m', type=float, default=0.005)
    parser.add_argument('--team', default='blue')
    parser.add_argument('--target', default='default')
    parser.add_argument('--target-x-m', type=float, default=1.05)
    parser.add_argument('--target-y-m', type=float, default=-0.15)
    parser.add_argument('--direct', type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument(
        '--config',
        default=str(Path(__file__).resolve().parents[1] / 'config' / 'pick_geometry.yaml'),
    )
    return parser.parse_args()


def _print_result(snapshot: dict[str, float], result: dict[str, float]) -> None:
    print('\n当前输入:')
    print('  sensor_3_mm = %.3f' % snapshot['sensor_3_mm'])
    print('  sensor_5_mm = %.3f' % snapshot['sensor_5_mm'])
    print('  odin_x_m = %.6f' % snapshot['odin_x_m'])
    print('  odin_y_m = %.6f' % snapshot['odin_y_m'])
    print('  odin_yaw_rad = %.6f' % snapshot['odin_yaw_rad'])
    print('  sensor_age_s = %.3f, pose_age_s = %.3f' % (
        snapshot['sensor_age_s'],
        snapshot['pose_age_s'],
    ))

    print('\n纠错结果:')
    print('  corrected_gripper_x_m = %.6f' % result['corrected_gripper_x_m'])
    print('  corrected_gripper_y_m = %.6f' % result['corrected_gripper_y_m'])
    print('  corrected_gripper_yaw_rad = %.6f' % result['corrected_gripper_yaw_rad'])
    print('  target_x_m = %.6f' % result['target_x_m'])
    print('  target_y_m = %.6f' % result['target_y_m'])
    print('  projection_x_m = %.6f' % result['target_projection_x_m'])
    print('  projection_y_m = %.6f' % result['target_projection_y_m'])
    print('  raw_forward_move_m = %.6f' % result['raw_gripper_forward_move_m'])
    print('  direct = %.1f' % result['direct'])
    print('  forward_move_m = %.6f' % result['gripper_forward_move_m'])
    print('  lateral_error_m = %.6f\n' % result['gripper_lateral_error_m'])


def main() -> None:
    args = _parse_args()
    (
        origin_x_m,
        origin_y_m,
        gripper_forward_m,
        gripper_left_m,
        gripper_yaw_offset_rad,
        _target_x_m,
        _target_y_m,
    ) = load_geometry(args.config, args.team, args.target)

    rclpy.init()
    node = InteractiveCorrectPrepare(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print('订阅 %s 和 %s' % (args.sensor_topic, args.pose_topic))
        print('平移服务: %s' % args.tool_service)
        print(
            'prepare 长度控制: command = base_length + move = %.3fm + move, '
            'range=[%.3f, %.3f]m'
            % (args.base_length_m, args.min_length_m, args.max_length_m)
        )
        while rclpy.ok():
            answer = input('是否纠错并计算夹爪位移？[y/N/q] ')
            if answer.strip().lower() in ('q', 'quit', 'exit'):
                break
            if not _yes(answer):
                continue

            snapshot = node.snapshot()
            if snapshot is None:
                continue

            result = correct_pose_from_odin(
                snapshot['sensor_3_mm'],
                snapshot['sensor_5_mm'],
                snapshot['odin_x_m'],
                snapshot['odin_y_m'],
                snapshot['odin_yaw_rad'],
                origin_x_m,
                origin_y_m,
                gripper_forward_m,
                gripper_left_m,
                gripper_yaw_offset_rad,
                args.target_x_m,
                args.target_y_m,
                args.direct,
            )
            _print_result(snapshot, result)

            move_m = result['gripper_forward_move_m']
            if abs(move_m) <= args.deadband_m:
                print(
                    '平移量 %.6fm 在 deadband %.6fm 内，不触发平移。'
                    % (move_m, args.deadband_m)
                )
                continue

            command_length_m = args.base_length_m + move_m
            print(
                '夹爪长度命令: base_length %.6fm + move %.6fm = %.6fm'
                % (args.base_length_m, move_m, command_length_m)
            )
            if (
                command_length_m < args.min_length_m
                or command_length_m > args.max_length_m
            ):
                print(
                    '长度 %.6fm 超出允许范围 [%.6f, %.6f]，不触发平移。'
                    % (
                        command_length_m,
                        args.min_length_m,
                        args.max_length_m,
                    )
                )
                continue

            answer = input(
                '是否触发平移 prepare(length=%.6fm)？[y/N] '
                % command_length_m
            )
            if not _yes(answer):
                print('已取消平移。')
                continue

            if not node.wait_for_tool_service():
                print('工具服务不可用: %s' % args.tool_service)
                continue
            node.call_prepare(command_length_m)
            time.sleep(0.1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
