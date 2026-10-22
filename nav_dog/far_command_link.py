#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive the laptop's final Goalpoint on TCP 8899 and publish /goal_point.

The Go2's FAR consumes /goal_point and generates /way_point itself. This bridge
never publishes /way_point, /path or a velocity command, so there is exactly one
planner and one execution chain.
"""

import argparse
import math
import os
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from far_command_protocol import (
    FRAME_HEADER, MAX_FRAME_BYTES, OP_FINAL_GOAL, decode_command_payload,
)


class FarCommandLink(Node):

    def __init__(self, port):
        super().__init__('far_command_link')
        self.port = int(port)
        self.closed = threading.Event()
        self.client = None
        self.client_lock = threading.Lock()
        self.last_point = None
        self.last_accept_time = 0.0
        self.received_count = 0
        self.published_count = 0
        self.dropped_count = 0
        self.min_interval = 0.2

        self.goal_pub = self.create_publisher(PointStamped, '/goal_point', 10)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

        self.acceptor = threading.Thread(target=self.accept_loop, daemon=True)
        self.acceptor.start()
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'Goalpoint channel listening on %d (final goal only)' % self.port)

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
            self.get_logger().info('laptop Goalpoint connected from %s:%d' % address)
            threading.Thread(
                target=self.reader_loop, args=(connection,), daemon=True).start()

    def reader_loop(self, connection):
        buf = bytearray()
        try:
            while not self.closed.is_set():
                data = connection.recv(4096)
                if not data:
                    break
                buf.extend(data)
                while len(buf) >= FRAME_HEADER.size:
                    length = FRAME_HEADER.unpack_from(buf)[0]
                    if length < 1 or length > MAX_FRAME_BYTES:
                        self.get_logger().warn(
                            'invalid Goalpoint frame length %d' % length)
                        return
                    end = FRAME_HEADER.size + length
                    if len(buf) < end:
                        break
                    payload = bytes(buf[FRAME_HEADER.size:end])
                    del buf[:end]
                    self.handle_payload(payload)
        except OSError:
            pass
        finally:
            with self.client_lock:
                if self.client is connection:
                    self.client = None
            try:
                connection.close()
            except OSError:
                pass
            self.get_logger().info('laptop Goalpoint disconnected')

    def handle_payload(self, payload):
        self.received_count += 1
        try:
            opcode, point = decode_command_payload(payload)
        except (ValueError, TypeError) as error:
            self.dropped_count += 1
            self.get_logger().warn('rejected Goalpoint: %s' % error)
            return
        if opcode != OP_FINAL_GOAL:
            self.dropped_count += 1
            return

        now = time.monotonic()
        duplicate = (
            self.last_point is not None and
            math.dist(point, self.last_point) < 0.01 and
            now - self.last_accept_time < 0.5
        )
        if duplicate or now - self.last_accept_time < self.min_interval:
            self.dropped_count += 1
            return

        self.last_point = point
        self.last_accept_time = now
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.point.x, message.point.y, message.point.z = point
        self.goal_pub.publish(message)
        self.published_count += 1
        self.get_logger().info(
            'Goalpoint accepted on Go2: x=%.3f y=%.3f z=%.3f' % point)

    def report_status(self):
        with self.client_lock:
            connected = self.client is not None
        self.get_logger().info(
            'laptop=%s received=%d published=%d dropped=%d' %
            ('connected' if connected else 'waiting', self.received_count,
             self.published_count, self.dropped_count))

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


def enabled_from_env():
    value = os.environ.get('FAR_COMMAND_ENABLE', '1').strip().lower()
    return value not in ('0', 'false', 'off', 'no')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--port', type=int,
        default=int(os.environ.get('FAR_COMMAND_PORT', '8899')))
    options, ros_args = parser.parse_known_args(args)
    if not enabled_from_env():
        print('Goalpoint channel disabled by FAR_COMMAND_ENABLE')
        return 0
    rclpy.init(args=ros_args)
    node = FarCommandLink(options.port)
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
