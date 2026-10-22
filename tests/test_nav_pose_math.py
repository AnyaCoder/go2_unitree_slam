#!/usr/bin/env python3
"""Tests for the pure rigid-transform math used by nav_pose_bridge."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from nav_pose_math import (  # noqa: E402
    IDENTITY_POSE,
    Pose,
    blend_pose,
    compose,
    correction_from_global,
    inverse,
    pose_distance,
    rotate_vector,
)


def yaw_pose(x, y, yaw):
    return Pose(x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def close(left, right, tolerance=1e-9):
    assert pose_distance(left, right) < tolerance, (left, right)
    dot = abs(sum(a * b for a, b in zip(left[3:], right[3:])))
    assert abs(dot - 1.0) < tolerance, (left[3:], right[3:])


def test_compose_and_inverse_round_trip():
    left = yaw_pose(1.0, 2.0, math.pi / 2.0)
    right = yaw_pose(1.0, 0.0, 0.0)
    result = compose(left, right)
    close(result, yaw_pose(1.0, 3.0, math.pi / 2.0))
    close(compose(result, inverse(result)), IDENTITY_POSE)
    close(compose(left, inverse(left)), IDENTITY_POSE)


def test_global_correction_maps_high_rate_pose():
    global_pose = yaw_pose(5.0, 2.0, math.pi / 2.0)
    local_pose = yaw_pose(1.0, 1.0, 0.2)
    correction = correction_from_global(global_pose, local_pose)
    close(compose(correction, local_pose), global_pose)


def test_rotate_vector_and_blend():
    rotated = rotate_vector(
        (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
        (1.0, 0.0, 0.0))
    assert abs(rotated[0]) < 1e-9
    assert abs(rotated[1] - 1.0) < 1e-9
    halfway = blend_pose(yaw_pose(0.0, 0.0, 0.0),
                         yaw_pose(2.0, 0.0, math.pi / 2.0), 0.5)
    close(halfway, yaw_pose(1.0, 0.0, math.pi / 4.0))
