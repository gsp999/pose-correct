from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml

from .models import PoseEstimate


@dataclass(frozen=True)
class FieldOrigin:
    """Field/world origin expressed in Odin coordinates."""

    x_m: float
    y_m: float


@dataclass(frozen=True)
class GripperOffset:
    """Gripper pick frame expressed in the robot body frame."""

    forward_m: float
    left_m: float
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class TargetPose:
    """Known target pose in the field/world frame."""

    x_m: float
    y_m: float
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class PickTarget:
    """Target coordinates expected by pick_action."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class TeamGeometry:
    field_origin: FieldOrigin
    targets: dict[str, TargetPose]


@dataclass(frozen=True)
class PickGeometryConfig:
    gripper: GripperOffset
    teams: dict[str, TeamGeometry]


def odin_to_field_pose(pose: PoseEstimate, origin: FieldOrigin) -> PoseEstimate:
    """Translate an Odin pose into the field/world frame."""
    return PoseEstimate(
        x=pose.x - origin.x_m,
        y=pose.y - origin.y_m,
        yaw=pose.yaw,
    )


def field_to_odin_pose(pose: PoseEstimate, origin: FieldOrigin) -> PoseEstimate:
    """Translate a field/world pose back into the Odin frame."""
    return PoseEstimate(
        x=pose.x + origin.x_m,
        y=pose.y + origin.y_m,
        yaw=pose.yaw,
    )


def target_to_pick_coordinates(
    *,
    robot_pose_field: PoseEstimate,
    target_pose_field: TargetPose,
    gripper: GripperOffset,
) -> PickTarget:
    """Convert a known field target into pick_action local coordinates.

    The robot body frame uses +forward along ``robot_pose_field.yaw`` and
    +left perpendicular to the left. pick_action receives ``x_m`` as the
    lateral left offset and ``y_m`` as the forward offset in the gripper frame.
    """
    dx = target_pose_field.x_m - robot_pose_field.x
    dy = target_pose_field.y_m - robot_pose_field.y
    yaw = robot_pose_field.yaw

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    target_forward = dx * cos_yaw + dy * sin_yaw
    target_left = -dx * sin_yaw + dy * cos_yaw

    delta_forward = target_forward - gripper.forward_m
    delta_left = target_left - gripper.left_m

    cos_gripper = math.cos(gripper.yaw_rad)
    sin_gripper = math.sin(gripper.yaw_rad)
    gripper_forward = delta_forward * cos_gripper + delta_left * sin_gripper
    gripper_left = -delta_forward * sin_gripper + delta_left * cos_gripper

    return PickTarget(
        x_m=gripper_left,
        y_m=gripper_forward,
        yaw_rad=target_pose_field.yaw_rad - robot_pose_field.yaw - gripper.yaw_rad,
    )


def load_pick_geometry_config(path: str | Path) -> PickGeometryConfig:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    gripper_data = _require_mapping(data, "gripper")
    teams_data = _require_mapping(data, "teams")

    teams: dict[str, TeamGeometry] = {}
    for team, team_data in teams_data.items():
        if not isinstance(team_data, dict):
            raise ValueError(f"teams.{team} must be a mapping")
        origin_data = _require_mapping(team_data, "field_origin_in_odin")
        targets_data = _require_mapping(team_data, "targets")
        teams[str(team)] = TeamGeometry(
            field_origin=FieldOrigin(
                x_m=float(origin_data["x_m"]),
                y_m=float(origin_data["y_m"]),
            ),
            targets={
                str(name): _parse_target_pose(
                    value, f"teams.{team}.targets.{name}"
                )
                for name, value in targets_data.items()
            },
        )

    return PickGeometryConfig(
        gripper=GripperOffset(
            forward_m=float(gripper_data["forward_m"]),
            left_m=float(gripper_data["left_m"]),
            yaw_rad=float(gripper_data.get("yaw_rad", 0.0)),
        ),
        teams=teams,
    )


def _parse_target_pose(value: Any, name: str) -> TargetPose:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return TargetPose(
        x_m=float(value["x_m"]),
        y_m=float(value["y_m"]),
        yaw_rad=float(value.get("yaw_rad", 0.0)),
    )


def _require_mapping(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be a mapping")
    return value
