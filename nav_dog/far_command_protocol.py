#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP 8899 protocol: one map-frame final Goalpoint, no velocity fields."""

import math
import struct

FRAME_HEADER = struct.Struct('<I')
OP_FINAL_GOAL = 1
MAX_FRAME_BYTES = 64
GOAL_PAYLOAD = struct.Struct('<3d')


def pack_goal_frame(x, y, z=0.0):
    values = (float(x), float(y), float(z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Goalpoint contains a non-finite value')
    body = bytes((OP_FINAL_GOAL,)) + GOAL_PAYLOAD.pack(*values)
    return FRAME_HEADER.pack(len(body)) + body


def decode_command_payload(payload):
    """Return (opcode, (x, y, z)) or raise ValueError."""
    if not payload:
        raise ValueError('empty command payload')
    opcode = payload[0]
    body = payload[1:]
    if opcode != OP_FINAL_GOAL:
        raise ValueError('unsupported command opcode %d' % opcode)
    if len(body) != GOAL_PAYLOAD.size:
        raise ValueError('invalid Goalpoint payload length %d' % len(body))
    values = GOAL_PAYLOAD.unpack(body)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Goalpoint contains a non-finite value')
    return opcode, values
