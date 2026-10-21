#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compact laptop-to-Go2 global-path wire format.

The laptop owns global planning on the fixed map. Only an ordered path in the
map frame crosses this channel; point clouds and velocity commands never do.
"""

import math
import struct

FRAME_HEADER = struct.Struct('<I')
OP_GLOBAL_PATH = 2
COMMAND_OPS = (OP_GLOBAL_PATH,)
MAX_PATH_POINTS = 512
PATH_HEADER = struct.Struct('<IH')  # sequence, point count
PATH_POINT = struct.Struct('<3f')   # x, y, z in the map frame
MAX_FRAME_BYTES = 1 + PATH_HEADER.size + MAX_PATH_POINTS * PATH_POINT.size


def pack_global_path_frame(points, sequence):
    """Encode an ordered map-frame path as one command frame."""
    points = list(points)
    if len(points) > MAX_PATH_POINTS:
        raise ValueError('path has %d points; maximum is %d' %
                         (len(points), MAX_PATH_POINTS))
    sequence = int(sequence) & 0xFFFFFFFF
    body = bytes((OP_GLOBAL_PATH,)) + PATH_HEADER.pack(sequence, len(points))
    chunks = [body]
    for point in points:
        if len(point) != 3:
            raise ValueError('path point must contain x, y, z')
        values = tuple(float(value) for value in point)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('path point contains a non-finite value')
        chunks.append(PATH_POINT.pack(*values))
    body = b''.join(chunks)
    return FRAME_HEADER.pack(len(body)) + body


def decode_command_payload(payload):
    """Return (opcode, (sequence, points)) or raise ValueError."""
    if not payload:
        raise ValueError('empty command payload')
    opcode = payload[0]
    body = payload[1:]
    if opcode not in COMMAND_OPS:
        raise ValueError('unsupported command opcode %d' % opcode)
    if len(body) < PATH_HEADER.size:
        raise ValueError('short global-path payload')
    sequence, count = PATH_HEADER.unpack_from(body)
    if count > MAX_PATH_POINTS:
        raise ValueError('path point count %d exceeds limit' % count)
    expected = PATH_HEADER.size + count * PATH_POINT.size
    if len(body) != expected:
        raise ValueError('global-path length %d != %d' % (len(body), expected))
    points = []
    offset = PATH_HEADER.size
    for _ in range(count):
        point = PATH_POINT.unpack_from(body, offset)
        if not all(math.isfinite(value) for value in point):
            raise ValueError('path point contains a non-finite value')
        points.append(point)
        offset += PATH_POINT.size
    return opcode, (sequence, points)
