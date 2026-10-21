#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wire definition for Go2 state telemetry and the minimal obstacle cloud."""

import struct
from collections import namedtuple

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

FRAME_HEADER = struct.Struct('<I')
OP_TOPIC = 1
OP_CLOUD = 2
MAX_FRAME_BYTES = 16 * 1024 * 1024

CLOUD_HEADER = struct.Struct('<3fI')
CLOUD_DELTA_SCALE = 100.0
CLOUD_MAX_POINTS = 12000
CLOUD_POINT_BYTES = 6


TopicSpec = namedtuple('TopicSpec', 'remote_topic local_topic msg_type hz sensor')


# Large static terrain stays on the laptop. These are state-only robot topics.
TOPICS = (
    TopicSpec('/lio_sam_ros2/mapping/re_location_odometry', '/state_estimation',
              Odometry, 30.0, False),
    TopicSpec('/tf', '/tf', TFMessage, 20.0, False),
    TopicSpec('/tf_static', '/tf_static', TFMessage, 2.0, False),
    TopicSpec('/joint_states', '/joint_states', JointState, 30.0, False),
)


def qos_for(spec):
    """Match source publishers while keeping the bridge queue shallow."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT if spec.sensor else ReliabilityPolicy.RELIABLE,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL
                    if spec.remote_topic == '/tf_static' else DurabilityPolicy.VOLATILE),
    )


def pack_frame(opcode, payload):
    body = bytes((opcode,)) + payload
    return FRAME_HEADER.pack(len(body)) + body


def encode_topic(index, message):
    from rclpy.serialization import serialize_message
    return pack_frame(OP_TOPIC, struct.pack('<H', index) + serialize_message(message))


def encode_cloud(origin, points):
    """Encode filtered map-frame points compactly relative to the robot."""
    origin = np.asarray(origin, dtype=np.float32).reshape(3)
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) > CLOUD_MAX_POINTS:
        keep = np.linspace(0, len(points) - 1, CLOUD_MAX_POINTS, dtype=np.int64)
        points = points[keep]
    delta = np.rint((points - origin) * CLOUD_DELTA_SCALE)
    delta = np.clip(delta, -32768, 32767).astype('<i2')
    body = CLOUD_HEADER.pack(float(origin[0]), float(origin[1]), float(origin[2]),
                             len(delta)) + delta.tobytes()
    return pack_frame(OP_CLOUD, body)


def decode_cloud(payload):
    """Decode a cloud payload and return (origin, Nx3 float32 points)."""
    if len(payload) < CLOUD_HEADER.size:
        raise ValueError('short cloud payload')
    cx, cy, cz, count = CLOUD_HEADER.unpack_from(payload)
    if count > CLOUD_MAX_POINTS:
        raise ValueError('cloud point count %d exceeds limit' % count)
    expected = CLOUD_HEADER.size + count * CLOUD_POINT_BYTES
    if len(payload) != expected:
        raise ValueError('cloud payload length %d != %d' % (len(payload), expected))
    delta = np.frombuffer(payload, dtype='<i2', count=count * 3,
                          offset=CLOUD_HEADER.size).reshape(-1, 3)
    points = delta.astype(np.float32) / CLOUD_DELTA_SCALE
    points += np.array([cx, cy, cz], dtype=np.float32)
    return (float(cx), float(cy), float(cz)), points
