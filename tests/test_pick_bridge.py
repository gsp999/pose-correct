import math

import pytest

from pose_correct import (
    FieldOrigin,
    GripperOffset,
    PoseEstimate,
    TargetPose,
    load_pick_geometry_config,
    odin_to_field_pose,
    target_to_pick_coordinates,
)


def test_odin_to_field_pose_only_translates_origin() -> None:
    pose = odin_to_field_pose(
        PoseEstimate(x=3.2, y=4.5, yaw=0.7),
        FieldOrigin(x_m=1.0, y_m=2.0),
    )

    assert pose.x == pytest.approx(2.2)
    assert pose.y == pytest.approx(2.5)
    assert pose.yaw == pytest.approx(0.7)


def test_target_to_pick_coordinates_with_forward_and_left_offsets() -> None:
    pick = target_to_pick_coordinates(
        robot_pose_field=PoseEstimate(x=1.0, y=2.0, yaw=0.0),
        target_pose_field=TargetPose(x_m=1.4, y_m=2.2, yaw_rad=0.1),
        gripper=GripperOffset(forward_m=0.3, left_m=0.05),
    )

    assert pick.y_m == pytest.approx(0.1)
    assert pick.x_m == pytest.approx(0.15)
    assert pick.yaw_rad == pytest.approx(0.1)


def test_target_to_pick_coordinates_respects_robot_yaw() -> None:
    pick = target_to_pick_coordinates(
        robot_pose_field=PoseEstimate(x=1.0, y=1.0, yaw=math.pi / 2.0),
        target_pose_field=TargetPose(x_m=1.0, y_m=1.5, yaw_rad=0.0),
        gripper=GripperOffset(forward_m=0.2, left_m=0.0),
    )

    assert pick.y_m == pytest.approx(0.3)
    assert pick.x_m == pytest.approx(0.0)


def test_load_pick_geometry_config_keeps_team_origins_separate(tmp_path) -> None:
    config = tmp_path / "pick_geometry.yaml"
    config.write_text(
        """
gripper:
  forward_m: 0.3
  left_m: 0.02
  yaw_rad: 0.0
teams:
  red:
    field_origin_in_odin:
      x_m: 1.0
      y_m: 2.0
    targets:
      default:
        x_m: 3.0
        y_m: 4.0
  blue:
    field_origin_in_odin:
      x_m: -1.0
      y_m: -2.0
    targets:
      default:
        x_m: 5.0
        y_m: 6.0
""",
        encoding="utf-8",
    )

    loaded = load_pick_geometry_config(config)

    assert loaded.gripper.forward_m == pytest.approx(0.3)
    assert loaded.teams["red"].field_origin.x_m == pytest.approx(1.0)
    assert loaded.teams["blue"].field_origin.x_m == pytest.approx(-1.0)
    assert loaded.teams["red"].targets["default"].x_m == pytest.approx(3.0)
    assert loaded.teams["blue"].targets["default"].x_m == pytest.approx(5.0)
