#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LEGACY / NOT STARTED: old final-goal-only TCP command format.

TCP 8899 now carries OP_GLOBAL_PATH from nav_path_protocol.py. This module is
retained only for historical reference and must not be used by the final chain.
"""

import struct

FRAME_HEADER = struct.Struct('<I')
OP_FINAL_GOAL = 1
COMMAND_OPS = (OP_FINAL_GOAL,)
MAX_FRAME_BYTES = 64
WAYPOINT_PAYLOAD = struct.Struct('<3d')


def pack_waypoint_frame(x, y, z, opcode=OP_FINAL_GOAL):
    if opcode != OP_FINAL_GOAL:
        raise ValueError('only final-goal commands are supported')
    body = bytes((opcode,)) + WAYPOINT_PAYLOAD.pack(float(x), float(y), float(z))
    return FRAME_HEADER.pack(len(body)) + body


def decode_command_payload(payload):
    """Return (opcode, decoded value) or raise ValueError for invalid input."""
    if not payload:
        raise ValueError('empty command payload')
    opcode = payload[0]
    body = payload[1:]
    if opcode not in COMMAND_OPS:
        raise ValueError('unsupported command opcode %d' % opcode)
    if len(body) != WAYPOINT_PAYLOAD.size:
        raise ValueError('invalid waypoint payload length %d' % len(body))
    return opcode, WAYPOINT_PAYLOAD.unpack(body)
