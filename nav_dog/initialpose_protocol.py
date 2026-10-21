#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wire format for the dedicated laptop-to-Go2 initial-pose channel."""

import struct

FRAME_HEADER = struct.Struct('<I')
MAX_FRAME_BYTES = 64
POSE_PAYLOAD = struct.Struct('<3d')  # x, y, yaw in the map frame


def pack_initial_pose_frame(x, y, yaw):
    body = POSE_PAYLOAD.pack(float(x), float(y), float(yaw))
    return FRAME_HEADER.pack(len(body)) + body


def decode_initial_pose_payload(payload):
    if len(payload) != POSE_PAYLOAD.size:
        raise ValueError('invalid initial-pose payload length %d' % len(payload))
    return POSE_PAYLOAD.unpack(payload)
