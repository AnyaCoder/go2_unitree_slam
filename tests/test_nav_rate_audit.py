#!/usr/bin/env python3
"""Pure tests for the read-only navigation rate audit."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from nav_rate_audit import AuditSpec, RateWindow, classify  # noqa: E402


def feed(window, count, hz, start=10.0, stamp_start=100.0):
    period = 1.0 / hz
    for index in range(count):
        window.add(stamp_start + index * period, start + index * period,
                   frame_id='map')


def test_arrival_and_source_rate_are_measured():
    window = RateWindow()
    feed(window, 51, 50.0)
    arrival = window.arrival_stats()
    source = window.source_stats()
    assert window.count == 51
    assert math.isclose(arrival['hz'], 50.0, rel_tol=1e-9)
    assert math.isclose(source['hz'], 50.0, rel_tol=1e-9)
    assert window.age(11.0) == 0.0


def test_strict_window_detects_low_rate_only_when_required():
    spec = AuditSpec('/nav/state_estimation', 'nav_msgs/msg/Odometry',
                     40.0, 60.0, required=True)
    window = RateWindow()
    feed(window, 21, 20.0)
    assert classify(spec, window, 11.1, strict=True) == 'FAIL'
    assert classify(spec, window, 11.1, strict=False) == 'LOW'


def test_optional_motion_topic_can_be_absent():
    spec = AuditSpec('/cmd_vel', 'geometry_msgs/msg/Twist', required=False)
    assert classify(spec, RateWindow(), 10.0, strict=True) == 'OPTIONAL'
