#!/usr/bin/env python3
"""Tests for the fixed-PCD terrain source used on the real dog."""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from static_map_replay import load_pcd, voxel_downsample, window_points  # noqa: E402


def write_ascii_pcd(path, points):
    rows = '\n'.join('%.6f %.6f %.6f' % tuple(point) for point in points)
    path.write_text(
        '# .PCD v0.7\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        'WIDTH %d\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        'POINTS %d\n'
        'DATA ascii\n%s\n' % (len(points), len(points), rows),
        encoding='ascii')


def test_load_and_deterministic_voxel_downsample(tmp_path):
    points = np.array([
        [0.01, 0.01, 0.01],
        [0.02, 0.02, 0.02],
        [1.01, 0.01, 0.01],
    ])
    path = tmp_path / 'map.pcd'
    write_ascii_pcd(path, points)
    loaded = load_pcd(str(path))
    sampled = voxel_downsample(loaded, 0.5)
    assert loaded.shape == (3, 3)
    assert sampled.shape == (2, 3)
    assert np.array_equal(sampled, voxel_downsample(loaded, 0.5))


def test_window_is_exactly_six_by_six_metres():
    points = np.array([
        [0.0, 0.0, 0.0],
        [2.99, 2.99, 0.0],
        [3.01, 0.0, 0.0],
        [0.0, 3.01, 0.0],
        [0.0, 0.0, 100.0],
    ])
    kept = window_points(points, (0.0, 0.0, 0.0), 6.0)
    assert np.array_equal(kept, points[[0, 1, 4]])
