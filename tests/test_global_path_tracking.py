#!/usr/bin/env python3
"""Pure path-tracking tests for the Go2 global-path receiver."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from global_path_receiver import (  # noqa: E402
    choose_path_target, sensor_pose_to_body_pose,
)


def close(first, second, tolerance=1e-6):
    assert math.dist(first, second) <= tolerance, (first, second)


def test_straight_path_uses_forward_lookahead():
    target = choose_path_target(
        [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)], (0.0, 0.0, 0.0),
        lookahead=1.2)
    close(target, (1.2, 0.0, 0.0))


def test_sharp_corner_stops_at_vertex_before_lookahead():
    target = choose_path_target(
        [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0), (0.4, 3.0, 0.0)],
        (0.0, 0.0, 0.0), lookahead=1.2)
    close(target, (0.4, 0.0, 0.0))


def test_nearest_segment_tracks_progress_along_path():
    target = choose_path_target(
        [(0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (8.0, 0.0, 0.0)],
        (2.5, 0.0, 0.0), lookahead=1.0)
    close(target, (3.5, 0.0, 0.0))


def test_sensor_offset_matches_localplanner_pose_convention():
    body_x, body_y, body_z = sensor_pose_to_body_pose(
        2.0, 3.0, 0.4, math.pi / 2.0, 0.3, 0.0)
    close((body_x, body_y, body_z), (2.0, 2.7, 0.4))


def test_empty_path_has_no_target():
    assert choose_path_target([], (0.0, 0.0, 0.0)) is None


if __name__ == '__main__':
    test_straight_path_uses_forward_lookahead()
    test_sharp_corner_stops_at_vertex_before_lookahead()
    test_nearest_segment_tracks_progress_along_path()
    test_sensor_offset_matches_localplanner_pose_convention()
    test_empty_path_has_no_target()
    print('global path tracking tests passed')
