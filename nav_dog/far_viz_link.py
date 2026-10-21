#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stream Go2 state, local execution path and an optional compact cloud.

The laptop owns FAR on the fixed map; the Go2 owns local planning/execution.
This uplink carries robot state, TF/joints, the Go2 local path and a compact
near-field display cloud. FAR markers and /way_point never cross this link.
"""

import argparse
import os
import queue
import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from far_viz_protocol import (
    MAX_FRAME_BYTES, TOPICS, encode_cloud, encode_topic, qos_for,
)

CLOUD_TOPIC = '/lio_sam_ros2/mapping/lasercloud_in_world'
CLOUD_HZ = 1.0
CLOUD_RADIUS = 6.0
CLOUD_VOXEL = 0.15
CLOUD_Z_BELOW = 0.65
CLOUD_Z_ABOVE = 1.20


def cloud_to_xyz(message):
    """PointCloud2 -> float32 Nx3 using the serialized x/y/z field offsets."""
    count = int(message.width) * int(message.height)
    if count == 0 or message.point_step == 0:
        return None
    offsets = {field.name: int(field.offset) for field in message.fields}
    if not all(name in offsets for name in ('x', 'y', 'z')):
        return None
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    count = min(count, raw.size // message.point_step)
    if count == 0:
        return None
    rows = raw[:count * message.point_step].reshape(count, message.point_step)
    points = np.empty((count, 3), dtype=np.float32)
    for index, name in enumerate(('x', 'y', 'z')):
        offset = offsets[name]
        points[:, index] = rows[:, offset:offset + 4].copy().view(np.float32).ravel()
    return points[np.isfinite(points).all(axis=1)]


class FarVizLink(Node):

    def __init__(self, port, cloud_enabled=True):
        super().__init__('far_viz_link')
        self.port = int(port)
        self.cloud_enabled = bool(cloud_enabled)
        self.client = None
        self.client_lock = threading.Lock()
        self.outbound = queue.Queue(maxsize=32)
        self.closed = threading.Event()
        self.last_sent = [None] * len(TOPICS)
        self.last_callback_time = [0.0] * len(TOPICS)
        self.topic_counts = [0] * len(TOPICS)
        self.last_cloud_time = 0.0
        self.latest_pose = None
        self.drop_count = 0
        self.cloud_count = 0

        self.subscribers = []
        for index, spec in enumerate(TOPICS):
            self.subscribers.append(self.create_subscription(
                spec.msg_type, spec.remote_topic,
                lambda msg, i=index: self.topic_callback(i, msg),
                qos_for(spec)))

        if self.cloud_enabled:
            cloud_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT)
            self.cloud_subscription = self.create_subscription(
                PointCloud2, CLOUD_TOPIC, self.cloud_callback, cloud_qos)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)
        self.writer = threading.Thread(target=self.writer_loop, daemon=True)
        self.acceptor = threading.Thread(target=self.accept_loop, daemon=True)
        self.writer.start()
        self.acceptor.start()
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'state link listening on %d; cloud=%s (%s)' %
            (self.port, 'on' if self.cloud_enabled else 'off', CLOUD_TOPIC))

    def connected(self):
        with self.client_lock:
            return self.client is not None

    def enqueue(self, payload):
        try:
            self.outbound.put_nowait(payload)
        except queue.Full:
            self.drop_count += 1
            try:
                self.outbound.get_nowait()
                self.outbound.put_nowait(payload)
            except queue.Empty:
                pass

    def topic_callback(self, index, message):
        if index == 0:
            position = message.pose.pose.position
            pose = (float(position.x), float(position.y), float(position.z))
            if all(np.isfinite(value) for value in pose):
                self.latest_pose = pose

        if not self.connected():
            return
        spec = TOPICS[index]
        now = time.monotonic()
        if now - self.last_callback_time[index] < 1.0 / spec.hz:
            return
        self.last_callback_time[index] = now

        payload = encode_topic(index, message)
        if len(payload) > MAX_FRAME_BYTES:
            self.get_logger().warn('drop oversized %s frame: %d bytes' %
                                   (spec.remote_topic, len(payload)))
            return
        if spec.remote_topic not in ('/tf', '/tf_static') and payload == self.last_sent[index]:
            return
        self.last_sent[index] = payload
        self.topic_counts[index] += 1
        self.enqueue(payload)

    def cloud_callback(self, message):
        if not self.cloud_enabled or not self.connected():
            return
        now = time.monotonic()
        if now - self.last_cloud_time < 1.0 / CLOUD_HZ:
            return
        self.last_cloud_time = now
        pose = self.latest_pose
        points = cloud_to_xyz(message)
        if pose is None or points is None or len(points) == 0:
            return

        origin = np.asarray(pose, dtype=np.float32)
        dx = points[:, 0] - origin[0]
        dy = points[:, 1] - origin[1]
        dz = points[:, 2] - origin[2]
        keep = (
            (dx * dx + dy * dy <= CLOUD_RADIUS * CLOUD_RADIUS) &
            (dz >= -CLOUD_Z_BELOW) & (dz <= CLOUD_Z_ABOVE)
        )
        points = points[keep]
        if len(points) == 0:
            return

        keys = np.floor((points - origin) / CLOUD_VOXEL).astype(np.int64)
        _, unique = np.unique(keys, axis=0, return_index=True)
        points = points[unique]
        payload = encode_cloud(origin, points)
        # Send at the configured low rate even when the set is unchanged: the
        # laptop treats this stream as a lease and expires stale obstacles.
        self.cloud_count += 1
        self.enqueue(payload)

    def accept_loop(self):
        while not self.closed.is_set():
            try:
                connection, address = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.client_lock:
                old = self.client
                self.client = connection
            if old is not None:
                try:
                    old.shutdown(socket.SHUT_RDWR)
                    old.close()
                except OSError:
                    pass
            while not self.outbound.empty():
                try:
                    self.outbound.get_nowait()
                except queue.Empty:
                    break
            self.get_logger().info('laptop state connected from %s:%d' % address)
            threading.Thread(
                target=self.reader_loop, args=(connection,), daemon=True).start()

    def writer_loop(self):
        while not self.closed.is_set():
            try:
                frame = self.outbound.get(timeout=0.5)
            except queue.Empty:
                continue
            with self.client_lock:
                connection = self.client
            if connection is None:
                continue
            try:
                connection.sendall(frame)
            except OSError:
                self.disconnect(connection)

    def reader_loop(self, connection):
        """Reject and discard all inbound bytes on the state channel."""
        try:
            while not self.closed.is_set():
                if not connection.recv(4096):
                    break
        except OSError:
            pass
        finally:
            self.disconnect(connection)

    def disconnect(self, connection):
        with self.client_lock:
            if self.client is connection:
                self.client = None
        try:
            connection.close()
        except OSError:
            pass
        self.get_logger().info('laptop state disconnected')

    def report_status(self):
        with self.client_lock:
            connected = self.client is not None
        with self.outbound.mutex:
            queued = len(self.outbound.queue)
        def count(topic):
            return next((self.topic_counts[i] for i, spec in enumerate(TOPICS)
                         if spec.remote_topic == topic), 0)
        self.get_logger().info(
            'laptop=%s queue=%d dropped=%d odom=%d path=%d '
            'obstacle_frames=%d' %
            ('connected' if connected else 'waiting', queued,
             self.drop_count,
             count('/lio_sam_ros2/mapping/re_location_odometry'),
             count('/path'), self.cloud_count))

    def close(self):
        self.closed.set()
        try:
            self.server.close()
        except OSError:
            pass
        with self.client_lock:
            connection = self.client
            self.client = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
                connection.close()
            except OSError:
                pass


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8898)
    parser.add_argument('--no-cloud', dest='cloud', action='store_false')
    parser.set_defaults(
        cloud=os.environ.get('FAR_VIZ_CLOUD_ENABLE', '1').strip().lower()
        not in ('0', 'false', 'off', 'no'))
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = FarVizLink(options.port, options.cloud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
