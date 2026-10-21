#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive Go2 pose/TF/joint telemetry and the optional minimal obstacle cloud.

The laptop owns all large, static terrain data. Raw scans, camera data, FAR
markers and paths are never accepted here; waypoints use a separate channel.
"""

import argparse
import os
import socket
import struct
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'nav_dog'))
from far_viz_protocol import (  # noqa: E402
    FRAME_HEADER, MAX_FRAME_BYTES, OP_CLOUD, OP_TOPIC, TOPICS, decode_cloud,
)


def read_pcd(path):
    """Read binary PCD x/y/z and optional intensity into a compact cloud."""
    with open(path, 'rb') as stream:
        lines = []
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError('PCD missing DATA line: %s' % path)
            text = line.decode('ascii', 'replace').strip()
            if text:
                lines.append(text)
            if text.startswith('DATA'):
                break
        fields = sizes = types = counts = None
        points = 0
        binary = False
        for line in lines:
            items = line.split()
            if not items:
                continue
            key = items[0]
            if key == 'FIELDS':
                fields = items[1:]
            elif key == 'SIZE':
                sizes = [int(v) for v in items[1:]]
            elif key == 'TYPE':
                types = items[1:]
            elif key == 'COUNT':
                counts = [int(v) for v in items[1:]]
            elif key == 'POINTS':
                points = int(items[1])
            elif key == 'DATA':
                binary = items[1] == 'binary'
        if not binary or fields is None or sizes is None or types is None:
            raise RuntimeError('only binary PCD is supported: %s' % path)
        if counts is None:
            counts = [1] * len(fields)
        kind = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = []
        for name, size, typ, count in zip(fields, sizes, types, counts):
            code = '<' + kind[typ] + str(size)
            dtype.append((name, code, (count,)) if count > 1 else (name, code))
        raw = stream.read(np.dtype(dtype).itemsize * points)
    table = np.frombuffer(raw, dtype=np.dtype(dtype), count=points)
    xyz = np.column_stack((table['x'], table['y'], table['z'])).astype(np.float32)
    intensity = (np.asarray(table['intensity'], dtype=np.float32)
                 if 'intensity' in fields else np.zeros(points, dtype=np.float32))
    return xyz, intensity


def make_cloud(xyz, intensity, frame_id, stamp):
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(xyz)
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * len(xyz)
    data = np.empty((len(xyz), 4), dtype=np.float32)
    data[:, :3] = xyz
    data[:, 3] = intensity
    cloud.data = data.tobytes()
    cloud.is_dense = True
    return cloud

def restamp(message, stamp):
    if isinstance(message, TFMessage):
        for transform in message.transforms:
            transform.header.stamp = stamp
    elif hasattr(message, 'header'):
        message.header.stamp = stamp
        if hasattr(message, 'poses'):
            for pose in message.poses:
                pose.header.stamp = stamp
    return message


class FarVizReceiver(Node):

    def __init__(self, host, port, map_path, tf_child='sensor'):
        super().__init__('far_viz_receiver')
        self.host = host
        self.port = port
        self.closed = threading.Event()
        self.connection = None
        self.send_lock = threading.Lock()
        self.static_cloud = None
        self.packet_count = 0
        self.topic_counts = [0] * len(TOPICS)
        self.state_count = 0
        self.cloud_count = 0
        self.last_connect_error = None
        self.tf_child = tf_child

        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.topic_publishers = []
        for spec in TOPICS:
            qos = static_qos if spec.local_topic == '/tf_static' else reliable
            self.topic_publishers.append(
                self.create_publisher(spec.msg_type, spec.local_topic, qos))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.min_cloud_pub = self.create_publisher(
            PointCloud2, '/go2/min_obstacle_cloud', 2)
        self.added_obstacles_pub = self.create_publisher(
            PointCloud2, '/added_obstacles', 5)

        if map_path:
            xyz, intensity = read_pcd(map_path)
            self.static_cloud = make_cloud(
                xyz, intensity, 'map', self.get_clock().now().to_msg())
            self.map_pub = self.create_publisher(PointCloud2, '/overall_map', 2)
            self.create_timer(1.0, self.publish_map)
            self.get_logger().info('static map: %d points from %s' %
                                   (len(xyz), map_path))

        self.create_timer(5.0, self.report_status)
        threading.Thread(target=self.receive_loop, daemon=True).start()

    def publish_map(self):
        if self.static_cloud is not None:
            self.static_cloud.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.static_cloud)

    def receive_loop(self):
        while not self.closed.is_set():
            try:
                connection = socket.create_connection((self.host, self.port), timeout=3.0)
            except OSError as error:
                message = '%s: %s' % (type(error).__name__, error)
                if message != self.last_connect_error:
                    self.get_logger().warn('cannot connect to Go2 state link: %s' % message)
                    self.last_connect_error = message
                self.closed.wait(2.0)
                continue
            self.last_connect_error = None
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.settimeout(None)
            with self.send_lock:
                self.connection = connection
            self.get_logger().info('connected to Go2 FAR view at %s:%d' %
                                   (self.host, self.port))
            buf = bytearray()
            try:
                while not self.closed.is_set():
                    data = connection.recv(262144)
                    if not data:
                        break
                    buf.extend(data)
                    while len(buf) >= FRAME_HEADER.size:
                        length = FRAME_HEADER.unpack_from(buf)[0]
                        if length < 2 or length > MAX_FRAME_BYTES:
                            raise RuntimeError('bad frame length %d' % length)
                        end = FRAME_HEADER.size + length
                        if len(buf) < end:
                            break
                        opcode = buf[FRAME_HEADER.size]
                        payload = bytes(buf[FRAME_HEADER.size + 1:end])
                        del buf[:end]
                        if opcode == OP_TOPIC:
                            self.on_topic_frame(payload)
                        elif opcode == OP_CLOUD:
                            self.on_cloud_frame(payload)
                        else:
                            self.get_logger().warn(
                                'ignored state frame opcode %d' % opcode)
            except (OSError, RuntimeError, ValueError) as error:
                self.get_logger().warn('robot link: %s' % error)
            finally:
                with self.send_lock:
                    if self.connection is connection:
                        self.connection = None
                try:
                    connection.close()
                except OSError:
                    pass
            self.closed.wait(1.0)

    def on_topic_frame(self, payload):
        if len(payload) < 2:
            return
        index = struct.unpack_from('<H', payload)[0]
        if index >= len(TOPICS):
            return
        spec = TOPICS[index]
        message = deserialize_message(payload[2:], spec.msg_type)
        restamp(message, self.get_clock().now().to_msg())
        self.topic_publishers[index].publish(message)
        self.packet_count += 1
        self.topic_counts[index] += 1
        if spec.local_topic == '/state_estimation' and isinstance(message, Odometry):
            self.state_count += 1
            if self.state_count == 1:
                self.get_logger().info('first SLAM odometry received; publishing map -> vehicle')
            self.publish_vehicle_tf(message)

    def on_cloud_frame(self, payload):
        try:
            _, points = decode_cloud(payload)
        except ValueError as error:
            self.get_logger().warn('invalid minimal obstacle cloud: %s' % error)
            return
        cloud = make_cloud(
            points, np.zeros(len(points), dtype=np.float32), 'map',
            self.get_clock().now().to_msg())
        self.min_cloud_pub.publish(cloud)
        self.added_obstacles_pub.publish(cloud)
        self.cloud_count += 1
        if self.cloud_count == 1:
            self.get_logger().info(
                'first minimal obstacle cloud: %d points; '
                'published to /go2/min_obstacle_cloud and /added_obstacles' % len(points))

    def publish_vehicle_tf(self, odometry):
        transform = TransformStamped()
        transform.header.stamp = odometry.header.stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = self.tf_child
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def report_status(self):
        state = 'connected' if self.connection is not None else 'reconnecting'
        def count(topic):
            return next((self.topic_counts[i] for i, item in enumerate(TOPICS)
                         if item.local_topic == topic), 0)
        self.get_logger().info(
            'robot=%s:%d %s packets=%d odom=%d tf=%d tf_static=%d joints=%d cloud=%d' %
            (self.host, self.port, state, self.packet_count,
             count('/state_estimation'), count('/tf'), count('/tf_static'),
             count('/joint_states'), self.cloud_count))

    def close(self):
        self.closed.set()
        with self.send_lock:
            connection = self.connection
            self.connection = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
                connection.close()
            except OSError:
                pass


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='192.168.76.1')
    parser.add_argument('--port', type=int, default=8898)
    parser.add_argument('--map', default='')
    parser.add_argument('--tf-child', default='sensor')
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = FarVizReceiver(options.host, options.port, options.map, options.tf_child)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
