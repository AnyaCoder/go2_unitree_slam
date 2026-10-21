#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wire format for the one-way laptop-to-Go2 waypoint command channel.

Both opcodes carry only a map-frame point:
  * final goal: selected by the operator in RViz
  * FAR waypoint: produced by the laptop FAR planner from the static map
"""

import struct

FRAME_HEADER = struct.Struct('<I')
OP_FINAL_GOAL = 1
OP_FAR_WAYPOINT = 2
WAYPOINT_OPS = (OP_FINAL_GOAL, OP_FAR_WAYPOINT)
MAX_FRAME_BYTES = 64
WAYPOINT_PAYLOAD = struct.Struct('<3d')


def pack_waypoint_frame(x, y, z, opcode=OP_FINAL_GOAL):
    if opcode not in WAYPOINT_OPS:
        raise ValueError('unsupported waypoint opcode %d' % opcode)
    body = bytes((opcode,)) + WAYPOINT_PAYLOAD.pack(float(x), float(y), float(z))
    return FRAME_HEADER.pack(len(body)) + body


def decode_command_payload(payload):
    """Return (opcode, decoded value) or raise ValueError for invalid input."""
    if not payload:
        raise ValueError('empty command payload')
    opcode = payload[0]
    body = payload[1:]
    if opcode not in WAYPOINT_OPS:
        raise ValueError('unsupported command opcode %d' % opcode)
    if len(body) != WAYPOINT_PAYLOAD.size:
        raise ValueError('invalid waypoint payload length %d' % len(body))
    return opcode, WAYPOINT_PAYLOAD.unpack(body)
