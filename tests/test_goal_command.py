#!/usr/bin/env python3
"""TCP 8899 final-Goalpoint protocol tests."""

import math
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from far_command_protocol import (  # noqa: E402
    FRAME_HEADER, OP_FINAL_GOAL, decode_command_payload, pack_goal_frame,
)


def payload_of(frame):
    return frame[FRAME_HEADER.size:]


def test_goal_round_trip():
    opcode, point = decode_command_payload(
        payload_of(pack_goal_frame(1.25, -3.5, 0.1)))
    assert opcode == OP_FINAL_GOAL
    assert point == (1.25, -3.5, 0.1)


def test_non_finite_goal_is_rejected():
    for value in (math.nan, math.inf, -math.inf):
        try:
            pack_goal_frame(value, 0.0, 0.0)
        except ValueError:
            pass
        else:
            raise AssertionError('non-finite goal accepted')


def test_unknown_opcode_is_rejected():
    payload = bytes((99,)) + struct.pack('<3d', 1.0, 2.0, 3.0)
    try:
        decode_command_payload(payload)
    except ValueError:
        pass
    else:
        raise AssertionError('unknown opcode accepted')
