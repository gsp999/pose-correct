#!/usr/bin/env python3
"""Correct field-frame pose from raw Odin pose and sensor 3/5 distances.

This standalone helper uses the low-residual model fitted from
``data_sensor_3_5_field_pose.csv``:

    m = sensor_3_mm / 1000
    n = sensor_5_mm / 1000
    theta = odin_yaw_rad - 1.6144295581

Input distances are millimeters. Input ``odin_x_m``/``odin_y_m`` are raw Odin
coordinates. The helper loads the selected team's field origin from
``config/pick_geometry.yaml`` and also reports the raw Odin pose translated
into that field-edge coordinate frame. Corrected x/y are meters in the same
field-edge coordinate frame. Output yaw keeps the original Odin yaw.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import yaml


YAW_OFFSET_RAD = -1.6144295581
M_SENSOR_X = -0.3739821743
M_SENSOR_Y = 0.0124127122
N_SENSOR_X = 0.0443469277
N_SENSOR_Y = -0.3087514918
DEFAULT_TARGET_X_M = 1.05
DEFAULT_TARGET_Y_M = -0.15
DEFAULT_DIRECT = 1.0


def load_geometry(
    config_path: str | Path = 'config/pick_geometry.yaml',
    team: str = 'blue',
    target: str = 'default',
) -> tuple[float, float, float, float, float, float, float]:
    """Return field origin, gripper geometry, and target from the YAML config."""
    with Path(config_path).open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    try:
        origin = data['teams'][team]['field_origin_in_odin']
        gripper = data['gripper']
        target_pose = data['teams'][team]['targets'][target]
        return (
            float(origin['x_m']),
            float(origin['y_m']),
            float(gripper['forward_m']),
            float(gripper['left_m']),
            float(gripper.get('yaw_rad', 0.0)),
            float(target_pose['x_m']),
            float(target_pose['y_m']),
        )
    except KeyError as exc:
        raise SystemExit(
            'missing field origin, gripper geometry, or target in %s '
            'for team %s target %s' % (config_path, team, target)
        ) from exc


def odin_to_field(
    odin_x_m: float,
    odin_y_m: float,
    field_origin_x_m: float,
    field_origin_y_m: float,
) -> tuple[float, float]:
    """Translate raw Odin x/y into the selected field-edge frame."""
    return (
        float(odin_x_m) - float(field_origin_x_m),
        float(odin_y_m) - float(field_origin_y_m),
    )


def correct_pose(
    sensor_3_mm: float,
    sensor_5_mm: float,
    odin_yaw_rad: float,
) -> tuple[float, float, float]:
    """Return corrected ``(x_m, y_m, yaw_rad)``.

    ``sensor_3_mm`` is the y-edge-facing ray distance and ``sensor_5_mm`` is
    the x-edge-facing ray distance. Distances are converted from mm to m.
    """
    m = float(sensor_3_mm) / 1000.0
    n = float(sensor_5_mm) / 1000.0
    theta = float(odin_yaw_rad) + YAW_OFFSET_RAD

    corrected_x_m = (
        m * math.cos(theta)
        - M_SENSOR_X * math.cos(theta)
        + M_SENSOR_Y * math.sin(theta)
    )
    corrected_y_m = (
        n * math.cos(theta)
        - N_SENSOR_X * math.sin(theta)
        - N_SENSOR_Y * math.cos(theta)
    )
    return corrected_x_m, corrected_y_m, float(odin_yaw_rad)


def robot_to_gripper_pose(
    robot_x_m: float,
    robot_y_m: float,
    robot_yaw_rad: float,
    gripper_forward_m: float,
    gripper_left_m: float,
    gripper_yaw_offset_rad: float,
) -> tuple[float, float, float]:
    """Transform corrected robot point S into the gripper pick pose."""
    yaw = float(robot_yaw_rad)
    gripper_x_m = (
        float(robot_x_m)
        + float(gripper_forward_m) * math.cos(yaw)
        - float(gripper_left_m) * math.sin(yaw)
    )
    gripper_y_m = (
        float(robot_y_m)
        + float(gripper_forward_m) * math.sin(yaw)
        + float(gripper_left_m) * math.cos(yaw)
    )
    gripper_yaw_rad = yaw + float(gripper_yaw_offset_rad)
    return gripper_x_m, gripper_y_m, gripper_yaw_rad


def gripper_forward_move_to_target(
    gripper_x_m: float,
    gripper_y_m: float,
    gripper_yaw_rad: float,
    target_x_m: float,
    target_y_m: float,
) -> tuple[float, float, float, float]:
    """Project the target onto the gripper yaw line.

    Returns ``(forward_move_m, lateral_error_m, projection_x_m,
    projection_y_m)``. Positive ``forward_move_m`` means moving along the
    gripper yaw direction.
    """
    dx = float(target_x_m) - float(gripper_x_m)
    dy = float(target_y_m) - float(gripper_y_m)
    yaw = float(gripper_yaw_rad)
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    left_x = -math.sin(yaw)
    left_y = math.cos(yaw)

    forward_move_m = dx * forward_x + dy * forward_y
    lateral_error_m = dx * left_x + dy * left_y
    projection_x_m = float(gripper_x_m) + forward_move_m * forward_x
    projection_y_m = float(gripper_y_m) + forward_move_m * forward_y
    return forward_move_m, lateral_error_m, projection_x_m, projection_y_m


def correct_pose_from_odin(
    sensor_3_mm: float,
    sensor_5_mm: float,
    odin_x_m: float,
    odin_y_m: float,
    odin_yaw_rad: float,
    field_origin_x_m: float,
    field_origin_y_m: float,
    gripper_forward_m: float,
    gripper_left_m: float,
    gripper_yaw_offset_rad: float,
    target_x_m: float = DEFAULT_TARGET_X_M,
    target_y_m: float = DEFAULT_TARGET_Y_M,
    direct: float = DEFAULT_DIRECT,
) -> dict[str, float]:
    """Correct pose from raw Odin coordinates and sensor distances.

    The returned dictionary includes both the raw Odin pose translated into
    the field frame and the sensor-corrected field-frame pose.
    """
    input_field_x_m, input_field_y_m = odin_to_field(
        odin_x_m,
        odin_y_m,
        field_origin_x_m,
        field_origin_y_m,
    )
    corrected_robot_x_m, corrected_robot_y_m, corrected_robot_yaw_rad = correct_pose(
        sensor_3_mm,
        sensor_5_mm,
        odin_yaw_rad,
    )
    gripper_x_m, gripper_y_m, gripper_yaw_rad = robot_to_gripper_pose(
        corrected_robot_x_m,
        corrected_robot_y_m,
        corrected_robot_yaw_rad,
        gripper_forward_m,
        gripper_left_m,
        gripper_yaw_offset_rad,
    )
    (
        gripper_forward_move_m,
        gripper_lateral_error_m,
        target_projection_x_m,
        target_projection_y_m,
    ) = gripper_forward_move_to_target(
        gripper_x_m,
        gripper_y_m,
        gripper_yaw_rad,
        target_x_m,
        target_y_m,
    )
    directed_gripper_forward_move_m = float(direct) * gripper_forward_move_m
    return {
        'input_field_x_m': input_field_x_m,
        'input_field_y_m': input_field_y_m,
        'input_field_yaw_rad': float(odin_yaw_rad),
        'corrected_robot_x_m': corrected_robot_x_m,
        'corrected_robot_y_m': corrected_robot_y_m,
        'corrected_robot_yaw_rad': corrected_robot_yaw_rad,
        'corrected_gripper_x_m': gripper_x_m,
        'corrected_gripper_y_m': gripper_y_m,
        'corrected_gripper_yaw_rad': gripper_yaw_rad,
        'target_x_m': float(target_x_m),
        'target_y_m': float(target_y_m),
        'target_projection_x_m': target_projection_x_m,
        'target_projection_y_m': target_projection_y_m,
        'gripper_forward_move_m': directed_gripper_forward_move_m,
        'raw_gripper_forward_move_m': gripper_forward_move_m,
        'direct': float(direct),
        'gripper_lateral_error_m': gripper_lateral_error_m,
        'robot_delta_x_m': corrected_robot_x_m - input_field_x_m,
        'robot_delta_y_m': corrected_robot_y_m - input_field_y_m,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Correct pose from raw Odin x/y/yaw and sensor 3/5 data.'
    )
    parser.add_argument('--sensor-3-mm', type=float)
    parser.add_argument('--sensor-5-mm', type=float)
    parser.add_argument('--odin-x-m', type=float)
    parser.add_argument('--odin-y-m', type=float)
    parser.add_argument('--odin-yaw-rad', type=float)
    parser.add_argument('--team', default='blue', help='Team key in config YAML.')
    parser.add_argument(
        '--target',
        default='default',
        help='Target key under teams.<team>.targets in config YAML.',
    )
    parser.add_argument(
        '--target-x-m',
        type=float,
        default=None,
        help='Target x in the selected field frame. Defaults to YAML, then 1.05.',
    )
    parser.add_argument(
        '--target-y-m',
        type=float,
        default=None,
        help='Target y in the selected field frame. Defaults to YAML, then -0.15.',
    )
    parser.add_argument(
        '--direct',
        type=float,
        default=DEFAULT_DIRECT,
        choices=(-1.0, 1.0),
        help='Direction multiplier for gripper_forward_move_m: 1 or -1.',
    )
    parser.add_argument(
        '--config',
        default='config/pick_geometry.yaml',
        help='Geometry YAML containing teams.<team>.field_origin_in_odin.',
    )
    parser.add_argument(
        '--input-csv',
        help=(
            'CSV with sensor_3_mm, sensor_5_mm, odin_x_m, odin_y_m, '
            'and odin_yaw_rad columns. odin_x_m/y_m must be raw Odin coords.'
        ),
    )
    parser.add_argument(
        '--output-csv',
        default='corrected_pose_sensor_3_5.csv',
        help='Output CSV path for --input-csv mode.',
    )
    return parser.parse_args()


def _correct_csv(
    input_csv: str,
    output_csv: str,
    *,
    field_origin_x_m: float,
    field_origin_y_m: float,
    gripper_forward_m: float,
    gripper_left_m: float,
    gripper_yaw_offset_rad: float,
    target_x_m: float,
    target_y_m: float,
    direct: float,
) -> None:
    input_path = Path(input_csv)
    output_path = Path(output_csv)
    with input_path.open('r', encoding='utf-8', newline='') as in_handle:
        reader = csv.DictReader(in_handle)
        if reader.fieldnames is None:
            raise SystemExit('input CSV has no header')
        required = {
            'sensor_3_mm',
            'sensor_5_mm',
            'odin_x_m',
            'odin_y_m',
            'odin_yaw_rad',
        }
        missing = required - set(reader.fieldnames)
        if missing:
            raise SystemExit('missing CSV columns: ' + ', '.join(sorted(missing)))

        fieldnames = list(reader.fieldnames) + [
            'input_field_x_m',
            'input_field_y_m',
            'input_field_yaw_rad',
            'corrected_robot_x_m',
            'corrected_robot_y_m',
            'corrected_robot_yaw_rad',
            'corrected_gripper_x_m',
            'corrected_gripper_y_m',
            'corrected_gripper_yaw_rad',
            'target_x_m',
            'target_y_m',
            'target_projection_x_m',
            'target_projection_y_m',
            'gripper_forward_move_m',
            'raw_gripper_forward_move_m',
            'direct',
            'gripper_lateral_error_m',
            'robot_delta_x_m',
            'robot_delta_y_m',
        ]
        with output_path.open('w', encoding='utf-8', newline='') as out_handle:
            writer = csv.DictWriter(out_handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in reader:
                result = correct_pose_from_odin(
                    float(row['sensor_3_mm']),
                    float(row['sensor_5_mm']),
                    float(row['odin_x_m']),
                    float(row['odin_y_m']),
                    float(row['odin_yaw_rad']),
                    field_origin_x_m,
                    field_origin_y_m,
                    gripper_forward_m,
                    gripper_left_m,
                    gripper_yaw_offset_rad,
                    target_x_m,
                    target_y_m,
                    direct,
                )
                for key, value in result.items():
                    row[key] = repr(value)
                writer.writerow(row)


def main() -> None:
    args = _parse_args()
    (
        origin_x_m,
        origin_y_m,
        gripper_forward_m,
        gripper_left_m,
        gripper_yaw_offset_rad,
        target_x_m,
        target_y_m,
    ) = load_geometry(args.config, args.team, args.target)
    if args.target_x_m is not None:
        target_x_m = args.target_x_m
    if args.target_y_m is not None:
        target_y_m = args.target_y_m
    if args.input_csv:
        _correct_csv(
            args.input_csv,
            args.output_csv,
            field_origin_x_m=origin_x_m,
            field_origin_y_m=origin_y_m,
            gripper_forward_m=gripper_forward_m,
            gripper_left_m=gripper_left_m,
            gripper_yaw_offset_rad=gripper_yaw_offset_rad,
            target_x_m=target_x_m,
            target_y_m=target_y_m,
            direct=args.direct,
        )
        print('wrote %s' % args.output_csv)
        return

    if (
        args.sensor_3_mm is None
        or args.sensor_5_mm is None
        or args.odin_x_m is None
        or args.odin_y_m is None
        or args.odin_yaw_rad is None
    ):
        raise SystemExit(
            'provide --input-csv or all of --sensor-3-mm, '
            '--sensor-5-mm, --odin-x-m, --odin-y-m, --odin-yaw-rad'
        )

    result = correct_pose_from_odin(
        args.sensor_3_mm,
        args.sensor_5_mm,
        args.odin_x_m,
        args.odin_y_m,
        args.odin_yaw_rad,
        origin_x_m,
        origin_y_m,
        gripper_forward_m,
        gripper_left_m,
        gripper_yaw_offset_rad,
        target_x_m,
        target_y_m,
        args.direct,
    )
    for key in (
        'input_field_x_m',
        'input_field_y_m',
        'input_field_yaw_rad',
        'corrected_robot_x_m',
        'corrected_robot_y_m',
        'corrected_robot_yaw_rad',
        'corrected_gripper_x_m',
        'corrected_gripper_y_m',
        'corrected_gripper_yaw_rad',
        'target_x_m',
        'target_y_m',
        'target_projection_x_m',
        'target_projection_y_m',
        'gripper_forward_move_m',
        'raw_gripper_forward_move_m',
        'direct',
        'gripper_lateral_error_m',
        'robot_delta_x_m',
        'robot_delta_y_m',
    ):
        print('%s=%.9f' % (key, result[key]))


if __name__ == '__main__':
    main()
