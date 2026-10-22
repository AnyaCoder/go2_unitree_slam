#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP 8898 protocol: Go2 state plus native FAR visualization messages.

The Go2 owns planning. This uplink never carries raw sensor clouds, motion
commands, or a second global path. Marker topics are intentionally rate-limited
so a full 3D view can be reconstructed on the laptop without measurable Wi-Fi
congestion.
"""

import struct
from collections import namedtuple

from geometry_msgs.msg import PointStamped, PolygonStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

FRAME_HEADER = struct.Struct('<I')
OP_TOPIC = 1
MAX_FRAME_BYTES = 16 * 1024 * 1024


TopicSpec = namedtuple('TopicSpec', 'remote_topic local_topic msg_type hz sensor')


TOPICS = (
    TopicSpec('/nav/state_estimation', '/state_estimation',
              Odometry, 30.0, False),
    TopicSpec('/tf', '/tf', TFMessage, 15.0, False),
    TopicSpec('/tf_static', '/tf_static', TFMessage, 1.0, False),
    TopicSpec('/joint_states', '/joint_states', JointState, 20.0, False),

    TopicSpec('/path', '/path', Path, 5.0, False),
    TopicSpec('/way_point', '/way_point', PointStamped, 5.0, False),
    TopicSpec('/navigation_boundary', '/navigation_boundary',
              PolygonStamped, 5.0, False),

    TopicSpec('/viz_path_topic', '/viz_path_topic', Marker, 2.0, False),
    TopicSpec('/viz_graph_topic', '/viz_graph_topic', MarkerArray, 1.0, False),
    TopicSpec('/viz_node_topic', '/viz_node_topic', MarkerArray, 2.0, False),
    TopicSpec('/viz_poly_topic', '/viz_poly_topic', MarkerArray, 1.0, False),
    TopicSpec('/viz_contour_topic', '/viz_contour_topic', MarkerArray, 1.0, False),
    TopicSpec('/viz_grid_map_topic', '/viz_grid_map_topic', MarkerArray, 0.5, False),
    TopicSpec('/viz_viewpoint_extend_topic', '/viz_viewpoint_extend_topic',
              MarkerArray, 1.0, False),
    TopicSpec('/far_robot_model', '/far_robot_model', MarkerArray, 1.0, False),
)


def qos_for(spec):
    """Match source publishers while keeping the bridge queue shallow."""
    if spec.remote_topic == '/tf_static':
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=(ReliabilityPolicy.BEST_EFFORT
                     if spec.sensor else ReliabilityPolicy.RELIABLE),
        durability=DurabilityPolicy.VOLATILE,
    )


def pack_frame(opcode, payload):
    body = bytes((opcode,)) + payload
    return FRAME_HEADER.pack(len(body)) + body


def encode_topic(index, message):
    from rclpy.serialization import serialize_message
    return pack_frame(OP_TOPIC, struct.pack('<H', index) + serialize_message(message))
