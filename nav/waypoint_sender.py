#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LEGACY / NOT STARTED: old /goal_point to Go2 FAR sender.

The final architecture keeps /goal_point local to the laptop FAR and sends the
resulting ordered path with global_path_sender.py. This file is retained only
for historical reference and is not part of the startup chain.
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
from far_command_protocol import OP_FINAL_GOAL, pack_waypoint_frame  # noqa: E402


class WaypointSender(Node):

    def __init__(self, host, port):
        super().__init__('go2_waypoint_sender')
        self.host = host
        self.port = int(port)
        self.closed = threading.Event()
        self.connection = None
        self.connection_lock = threading.Lock()
        self.outbound = queue.Queue(maxsize=8)
        self.last_goal = None
        self.last_accept_time = 0.0
        self.sent_count = 0
        self.drop_count = 0
        self.min_interval = 0.2

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            PointStamped, '/goal_point', self.on_goal, qos)
        self.create_timer(5.0, self.report_status)

        self.sender = threading.Thread(target=self.send_loop, daemon=True)
        self.sender.start()
        self.get_logger().info(
            'final-goal sender target %s:%d; waiting for /goal_point' %
            (self.host, self.port))

    def on_goal(self, message):
        x = float(message.point.x)
        y = float(message.point.y)
        z = float(message.point.z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            self.drop_count += 1
            self.get_logger().warn('drop non-finite final goal')
            return
        now = time.monotonic()
        duplicate = (
            self.last_goal is not None and
            math.dist((x, y, z), self.last_goal) < 0.01 and
            now - self.last_accept_time < 0.5
        )
        if duplicate or now - self.last_accept_time < self.min_interval:
            self.drop_count += 1
            return
        self.last_goal = (x, y, z)
        self.last_accept_time = now
        frame = pack_waypoint_frame(x, y, z, OP_FINAL_GOAL)
        try:
            self.outbound.put_nowait(frame)
        except queue.Full:
            self.drop_count += 1
            try:
                self.outbound.get_nowait()
                self.outbound.put_nowait(frame)
            except queue.Empty:
                pass
        self.get_logger().info(
            'final goal queued: x=%.3f y=%.3f z=%.3f' % (x, y, z))

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
                    'connected to Go2 goal channel %s:%d' % (self.host, self.port))
            try:
                frame = self.outbound.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                connection.sendall(frame)
                self.sent_count += 1
                self.get_logger().info('final goal sent to Go2')
            except OSError as error:
                self.get_logger().warn('final goal send failed: %s' % error)
                self.disconnect(connection)

    def disconnect(self, connection):
        with self.connection_lock:
            if self.connection is connection:
                self.connection = None
        try:
            connection.close()
        except OSError:
            pass
        self.get_logger().info('Go2 goal channel disconnected')

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
    parser.add_argument('--host', default=os.environ.get('FAR_VIZ_HOST_IP', '192.168.76.1'))
    parser.add_argument('--port', type=int,
                        default=int(os.environ.get('FAR_COMMAND_PORT', '8899')))
    parser.add_argument('--disable', action='store_true')
    options, ros_args = parser.parse_known_args(args)
    enabled = os.environ.get('GO2_WAYPOINT_ENABLE', '1').strip().lower()
    if options.disable or enabled in ('0', 'false', 'off', 'no'):
        print('waypoint sender disabled')
        return 0
    rclpy.init(args=ros_args)
    node = WaypointSender(options.host, options.port)
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
