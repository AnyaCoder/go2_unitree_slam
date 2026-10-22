#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Send RViz's final /goal_point to the Go2 over TCP 8899.

Only a new operator goal is transmitted. The sender does not replay an old goal
after reconnecting, resend a path, or publish any velocity command.
"""

import argparse
import math
import os
import queue
import socket
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'nav_dog'))
from far_command_protocol import pack_goal_frame  # noqa: E402


class GoalSender(Node):

    def __init__(self, host, port):
        super().__init__('go2_goal_sender')
        self.host = host
        self.port = int(port)
        self.closed = threading.Event()
        self.connection = None
        self.connection_lock = threading.Lock()
        self.outbound = queue.Queue(maxsize=8)
        self.last_point = None
        self.last_accept_time = 0.0
        self.sent_count = 0
        self.drop_count = 0

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointStamped, '/goal_point', self.on_goal, qos)
        self.create_timer(5.0, self.report_status)
        threading.Thread(target=self.send_loop, daemon=True).start()
        self.get_logger().info(
            'Goalpoint sender target %s:%d; waiting for RViz Goalpoint' %
            (self.host, self.port))

    def on_goal(self, message):
        frame = message.header.frame_id.strip('/')
        if frame not in ('', 'map'):
            self.drop_count += 1
            self.get_logger().warn('drop Goalpoint in frame %s' % frame)
            return
        values = (float(message.point.x), float(message.point.y),
                  float(message.point.z))
        if not all(math.isfinite(value) for value in values):
            self.drop_count += 1
            self.get_logger().warn('drop non-finite Goalpoint')
            return
        now = time.monotonic()
        duplicate = (
            self.last_point is not None and
            math.dist(values, self.last_point) < 0.01 and
            now - self.last_accept_time < 0.5
        )
        if duplicate:
            return
        with self.connection_lock:
            connected = self.connection is not None
        if not connected:
            self.drop_count += 1
            self.get_logger().warn(
                'drop Goalpoint because TCP 8899 is not connected; click again '
                'after the bridge reconnects')
            return

        self.last_point = values
        self.last_accept_time = now
        payload = pack_goal_frame(*values)
        try:
            self.outbound.put_nowait(payload)
        except queue.Full:
            self.drop_count += 1
            try:
                self.outbound.get_nowait()
                self.outbound.put_nowait(payload)
            except queue.Empty:
                pass
        self.get_logger().info(
            'RViz Goalpoint queued: x=%.3f y=%.3f z=%.3f' % values)

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
                    'connected to Go2 Goalpoint channel %s:%d' %
                    (self.host, self.port))
            try:
                frame = self.outbound.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                connection.sendall(frame)
                self.sent_count += 1
            except OSError as error:
                self.get_logger().warn('Goalpoint send failed: %s' % error)
                with self.connection_lock:
                    if self.connection is connection:
                        self.connection = None
                try:
                    connection.close()
                except OSError:
                    pass

    def report_status(self):
        with self.connection_lock:
            connected = self.connection is not None
        self.get_logger().info(
            'target=%s:%d %s sent=%d dropped=%d' %
            (self.host, self.port,
             'connected' if connected else 'reconnecting',
             self.sent_count, self.drop_count))

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
        default=int(os.environ.get('FAR_COMMAND_PORT', '8899')))
    parser.add_argument('--disable', action='store_true')
    options, ros_args = parser.parse_known_args(args)
    enabled = os.environ.get(
        'GO2_GOAL_ENABLE', os.environ.get('FAR_COMMAND_ENABLE', '1')
    ).strip().lower()
    if options.disable or enabled in ('0', 'false', 'off', 'no'):
        print('Goalpoint sender disabled')
        return 0
    rclpy.init(args=ros_args)
    node = GoalSender(options.host, options.port)
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
