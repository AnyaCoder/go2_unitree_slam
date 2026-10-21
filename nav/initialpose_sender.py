#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Forward RViz 2D Pose Estimate messages to the Go2 relocation input."""

import argparse
import math
import os
import queue
import socket
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'nav_dog'))
from initialpose_protocol import pack_initial_pose_frame  # noqa: E402


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class InitialPoseSender(Node):

    def __init__(self, host, port):
        super().__init__('go2_initialpose_sender')
        self.host = host
        self.port = int(port)
        self.closed = threading.Event()
        self.connection = None
        self.connection_lock = threading.Lock()
        self.outbound = queue.Queue(maxsize=4)
        self.last_pose = None
        self.last_accept_time = 0.0
        self.sent_count = 0
        self.drop_count = 0

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, qos)
        self.create_timer(5.0, self.report_status)
        threading.Thread(target=self.send_loop, daemon=True).start()
        self.get_logger().info(
            'initial-pose sender target %s:%d; waiting for RViz 2D Pose Estimate' %
            (self.host, self.port))

    def on_initial_pose(self, message):
        frame = message.header.frame_id
        if frame not in ('', 'map'):
            self.drop_count += 1
            self.get_logger().warn('drop initial pose in frame %s' % frame)
            return
        pose = message.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.drop_count += 1
            self.get_logger().warn('drop non-finite initial pose')
            return
        now = time.monotonic()
        duplicate = (
            self.last_pose is not None and
            math.dist((x, y), self.last_pose[:2]) < 0.01 and
            abs(math.remainder(yaw - self.last_pose[2], 2.0 * math.pi)) < 0.01 and
            now - self.last_accept_time < 0.5
        )
        if duplicate:
            return
        self.last_pose = (x, y, yaw)
        self.last_accept_time = now
        data = pack_initial_pose_frame(x, y, yaw)
        try:
            self.outbound.put_nowait(data)
        except queue.Full:
            self.drop_count += 1
            try:
                self.outbound.get_nowait()
                self.outbound.put_nowait(data)
            except queue.Empty:
                pass
        self.get_logger().info(
            'RViz initial pose queued: x=%.3f y=%.3f yaw=%.1f deg' %
            (x, y, math.degrees(yaw)))

    def send_loop(self):
        while not self.closed.is_set():
            with self.connection_lock:
                connection = self.connection
            if connection is None:
                try:
                    connection = socket.create_connection(
                        (self.host, self.port), timeout=3.0)
                except OSError:
                    self.closed.wait(1.0)
                    continue
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.connection_lock:
                    self.connection = connection
                self.get_logger().info(
                    'connected to Go2 initial-pose channel %s:%d' %
                    (self.host, self.port))
            try:
                data = self.outbound.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                connection.sendall(data)
                self.sent_count += 1
                self.get_logger().info('initial pose sent to Go2')
            except OSError as error:
                self.get_logger().warn('initial-pose send failed: %s' % error)
                self.disconnect(connection)

    def disconnect(self, connection):
        with self.connection_lock:
            if self.connection is connection:
                self.connection = None
        try:
            connection.close()
        except OSError:
            pass
        self.get_logger().info('Go2 initial-pose channel disconnected')

    def report_status(self):
        with self.connection_lock:
            connected = self.connection is not None
        with self.outbound.mutex:
            queued = len(self.outbound.queue)
        self.get_logger().info('target=%s:%d %s queued=%d sent=%d dropped=%d' %
                               (self.host, self.port,
                                'connected' if connected else 'reconnecting',
                                queued, self.sent_count, self.drop_count))

    def close(self):
        self.closed.set()
        with self.connection_lock:
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
    parser.add_argument(
        '--host', default=os.environ.get('FAR_VIZ_HOST_IP', '192.168.76.1'))
    parser.add_argument(
        '--port', type=int,
        default=int(os.environ.get('GO2_INITIALPOSE_PORT', '8900')))
    parser.add_argument('--disable', action='store_true')
    options, ros_args = parser.parse_known_args(args)
    enabled = os.environ.get('GO2_INITIALPOSE_ENABLE', '1').strip().lower()
    if options.disable or enabled in ('0', 'false', 'off', 'no'):
        print('initial-pose sender disabled')
        return 0
    rclpy.init(args=ros_args)
    node = InitialPoseSender(options.host, options.port)
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
