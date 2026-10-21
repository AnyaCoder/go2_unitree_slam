#!/usr/bin/env python3
"""Tests for laptop FAR marker -> wire path conversion."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav'))
sys.path.insert(0, str(ROOT / 'nav_dog'))

from global_path_sender import select_path_points  # noqa: E402


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def test_order_and_near_duplicates():
    points = [
        Point(0.0, 0.0, 0.0),
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
    ]
    assert select_path_points(points) == [
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]


def test_reduction_keeps_both_endpoints():
    points = [Point(float(index), 0.0, 0.0) for index in range(20)]
    selected = select_path_points(points, max_points=5)
    assert len(selected) == 5
    assert selected[0] == (0.0, 0.0, 0.0)
    assert selected[-1] == (19.0, 0.0, 0.0)


if __name__ == '__main__':
    test_order_and_near_duplicates()
    test_reduction_keeps_both_endpoints()
    print('global path sender tests passed')
