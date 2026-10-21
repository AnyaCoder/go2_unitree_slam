#!/usr/bin/env python3
"""Round-trip and bounds tests for the TCP 8899 global-path wire format."""

import math
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from nav_path_protocol import (  # noqa: E402
    FRAME_HEADER, MAX_FRAME_BYTES, MAX_PATH_POINTS, OP_GLOBAL_PATH,
    decode_command_payload, pack_global_path_frame,
)


def payload_of(frame):
    return frame[FRAME_HEADER.size:]


def test_empty_path_round_trip():
    opcode, (sequence, points) = decode_command_payload(
        payload_of(pack_global_path_frame([], 7)))
    assert opcode == OP_GLOBAL_PATH
    assert sequence == 7
    assert points == []


def test_three_point_order_round_trip():
    source = [(0.0, 1.0, 2.0), (3.5, 4.25, -0.5), (-1.0, 0.0, 0.75)]
    opcode, (sequence, points) = decode_command_payload(
        payload_of(pack_global_path_frame(source, 0xFFFFFFFF)))
    assert opcode == OP_GLOBAL_PATH
    assert sequence == 0xFFFFFFFF
    assert points == source


def test_over_limit_path_is_rejected():
    points = [(float(index), 0.0, 0.0) for index in range(MAX_PATH_POINTS + 1)]
    try:
        pack_global_path_frame(points, 1)
    except ValueError:
        pass
    else:
        raise AssertionError('path above MAX_PATH_POINTS must be rejected')


def test_maximum_path_fits_frame_limit():
    points = [(float(index), 0.0, 0.0) for index in range(MAX_PATH_POINTS)]
    frame = pack_global_path_frame(points, 1)
    length = FRAME_HEADER.unpack_from(frame)[0]
    assert length == len(frame) - FRAME_HEADER.size
    assert length <= MAX_FRAME_BYTES


def test_wrong_opcode_is_rejected():
    bad = bytes((99,)) + struct.pack('<IH', 1, 0)
    try:
        decode_command_payload(bad)
    except ValueError:
        pass
    else:
        raise AssertionError('wrong opcode must be rejected')


def test_truncated_payload_is_rejected():
    frame = pack_global_path_frame([(1.0, 2.0, 3.0)], 1)
    payload = payload_of(frame)
    try:
        decode_command_payload(payload[:-1])
    except ValueError:
        pass
    else:
        raise AssertionError('truncated path must be rejected')


def test_non_finite_path_is_rejected_both_ways():
    try:
        pack_global_path_frame([(math.inf, 0.0, 0.0)], 1)
    except ValueError:
        pass
    else:
        raise AssertionError('non-finite send point must be rejected')
    bad_body = bytes((OP_GLOBAL_PATH,)) + struct.pack('<IH', 1, 1)
    bad_body += struct.pack('<3f', math.nan, 0.0, 0.0)
    try:
        decode_command_payload(bad_body)
    except ValueError:
        pass
    else:
        raise AssertionError('non-finite receive point must be rejected')


if __name__ == '__main__':
    test_empty_path_round_trip()
    test_three_point_order_round_trip()
    test_over_limit_path_is_rejected()
    test_maximum_path_fits_frame_limit()
    test_wrong_opcode_is_rejected()
    test_truncated_payload_is_rejected()
    test_non_finite_path_is_rejected_both_ways()
    print('nav path protocol tests passed')
