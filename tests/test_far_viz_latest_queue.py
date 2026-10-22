#!/usr/bin/env python3
"""Latest-frame coalescing checks for the Go2 visualization uplink."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from far_viz_link import LatestFrameQueue  # noqa: E402


def test_same_topic_keeps_only_the_newest_frame():
    frames = LatestFrameQueue(2)
    frames.put(0, b'old')
    frames.put(0, b'older-still-pending')
    frames.put(0, b'new')

    assert frames.pending == 1
    assert frames.coalesced == 2
    assert frames.get(0.0) == (0, b'new')
    assert frames.pending == 0


def test_older_pending_topic_is_sent_before_newer_updates():
    frames = LatestFrameQueue(2)
    frames.put(0, b'state-1')
    frames.put(1, b'marker')
    frames.put(0, b'state-2')

    # The marker is the oldest pending item and cannot starve behind odometry.
    assert frames.get(0.0) == (1, b'marker')
    assert frames.get(0.0) == (0, b'state-2')


def test_clear_discards_frames_from_the_previous_connection():
    frames = LatestFrameQueue(1)
    frames.put(0, b'stale')
    frames.clear()
    assert frames.pending == 0
    assert frames.get(0.0) is None
