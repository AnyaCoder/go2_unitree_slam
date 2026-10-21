#!/usr/bin/env python3
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nav_dog'))

from far_command_link import sensor_pose_to_body_pose, yaw_from_quaternion


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


def assert_close(actual, expected):
    assert math.isclose(actual, expected, abs_tol=1e-9), (actual, expected)


def test_zero_heading_offset():
    body = sensor_pose_to_body_pose(2.0, 3.0, 0.4, 0.0, 0.3, 0.0)
    assert_close(body[0], 1.7)
    assert_close(body[1], 3.0)
    assert_close(body[2], 0.4)


def test_quarter_turn_offset():
    body = sensor_pose_to_body_pose(2.0, 3.0, 0.0, math.pi / 2.0, 0.3, 0.0)
    assert_close(body[0], 2.0)
    assert_close(body[1], 2.7)


def test_yaw_from_identity_and_quarter_turn():
    assert_close(yaw_from_quaternion(Quaternion()), 0.0)
    half = math.pi / 4.0
    assert_close(
        yaw_from_quaternion(Quaternion(z=math.sin(half), w=math.cos(half))),
        math.pi / 2.0)


def localplanner_vehicle_pose(sensor_x, sensor_y, yaw,
                               sensor_offset_x, sensor_offset_y):
    """Independent copy of upstream localPlanner.cpp:149-150 convention."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vehicle_x = (sensor_x - cos_yaw * sensor_offset_x +
                 sin_yaw * sensor_offset_y)
    vehicle_y = (sensor_y - sin_yaw * sensor_offset_x -
                 cos_yaw * sensor_offset_y)
    return vehicle_x, vehicle_y


def test_hold_target_equals_localplanner_vehicle_pose():
    # Before the first real FAR waypoint, far_command_link publishes this body
    # pose as /way_point.  Equality makes localPlanner's relative goal zero, so
    # no acceleration can be requested by the hold path.
    for yaw in (0.0, math.pi / 6.0, math.pi / 2.0, math.pi, -math.pi / 3.0):
        for sensor_x, sensor_y in ((1.0, 2.0), (-3.0, 5.0)):
            for offset_x, offset_y in ((0.16143, 0.0), (0.187, 0.02)):
                goal_x, goal_y, _ = sensor_pose_to_body_pose(
                    sensor_x, sensor_y, 0.0, yaw, offset_x, offset_y)
                vehicle_x, vehicle_y = localplanner_vehicle_pose(
                    sensor_x, sensor_y, yaw, offset_x, offset_y)
                assert_close(goal_x, vehicle_x)
                assert_close(goal_y, vehicle_y)


def test_mid360_model_offset_is_inside_pathfollower_stop_radius():
    # Upstream go2_nav pathFollower uses stopDisThre=0.3 m.  Even if model
    # baseline and the two independent mount references differ, all remain
    # inside that stop radius.
    for offset_x in (0.16143, 0.163, 0.187):
        assert math.hypot(offset_x, 0.0) < 0.3


if __name__ == '__main__':
    test_zero_heading_offset()
    test_quarter_turn_offset()
    test_yaw_from_identity_and_quarter_turn()
    test_hold_target_equals_localplanner_vehicle_pose()
    test_mid360_model_offset_is_inside_pathfollower_stop_radius()
    print('far_command pose tests passed')
