#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive map-frame waypoints on a dedicated laptop-to-Go2 TCP channel.

The laptop sends the operator's final goal and the FAR planner's current
waypoint as two distinct point-only opcodes. Velocity, pose initialization and
all other command types are deliberately unsupported.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from far_command_protocol import (
    FRAME_HEADER, MAX_FRAME_BYTES, OP_FAR_WAYPOINT, OP_FINAL_GOAL,
    decode_command_payload,
)


class FarCommandLink(Node):

    def __init__(self, port):
        super().__init__('far_command_link')
        self.port = int(port)
        self.closed = threading.Event()
        self.client = None
        self.client_lock = threading.Lock()
        self.last_points = {}
        self.last_accept_times = {}
        self.received_count = 0
        self.published_count = 0
        self.dropped_count = 0
        self.min_interval = 0.2
        self.goal_commands_ready = False
        self.ready_update_time = 0.0
        self.ready_rejected_count = 0

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/nav/goal_commands_ready', self.on_goal_commands_ready, latched)

        self.goal_pub = self.create_publisher(PointStamped, '/goal_point', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

        self.acceptor = threading.Thread(target=self.accept_loop, daemon=True)
        self.acceptor.start()
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'waypoint command channel listening on %d; waiting for safety gate' % self.port)

    def on_goal_commands_ready(self, message):
        ready = bool(message.data)
        self.ready_update_time = time.monotonic()
        if ready != self.goal_commands_ready:
            self.get_logger().info(
                'safety gate Goalpoint authorization: %s' %
                ('READY' if ready else 'BLOCKED'))
        self.goal_commands_ready = ready

    def _commands_ready_now(self):
        # Missing or stale safety status is fail-closed.  A crashed gate must
        # never leave a latched true value authorizing future waypoints.
        return self.goal_commands_ready and time.monotonic() - self.ready_update_time <= 2.0

    def accept_loop(self):
        while not self.closed.is_set():
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.client_lock:
                old = self.client
                self.client = conn
            if old is not None:
                try:
                    old.shutdown(socket.SHUT_RDWR)
                    old.close()
                except OSError:
                    pass
            self.get_logger().info('laptop command connected from %s:%d' % addr)
            threading.Thread(target=self.reader_loop, args=(conn,), daemon=True).start()

    def reader_loop(self, conn):
        buf = bytearray()
        try:
            while not self.closed.is_set():
                data = conn.recv(4096)
                if not data:
                    break
                buf.extend(data)
                while len(buf) >= FRAME_HEADER.size:
                    length = FRAME_HEADER.unpack_from(buf)[0]
                    if length < 1 or length > MAX_FRAME_BYTES:
                        self.get_logger().warn('invalid command frame length %d' % length)
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
                if self.client is conn:
                    self.client = None
            try:
                conn.close()
            except OSError:
                pass
            self.get_logger().info('laptop command disconnected')

    def handle_payload(self, payload):
        self.received_count += 1
        try:
            opcode, (x, y, z) = decode_command_payload(payload)
        except (ValueError, TypeError) as error:
            self.dropped_count += 1
            self.get_logger().warn('rejected command: %s' % error)
            return
        if not all(math.isfinite(value) for value in (x, y, z)):
            self.dropped_count += 1
            self.get_logger().warn('rejected non-finite waypoint')
            return
        if not self._commands_ready_now():
            self.dropped_count += 1
            self.ready_rejected_count += 1
            self.get_logger().warn(
                'rejected Goalpoint/FAR waypoint: IMU, initial pose or '
                'localization is not ready')
            return

        channel = 'goal' if opcode == OP_FINAL_GOAL else 'far_waypoint'
        now = time.monotonic()
        last_point = self.last_points.get(opcode)
        last_accept_time = self.last_accept_times.get(opcode, 0.0)
        duplicate = (
            last_point is not None and
            math.dist((x, y, z), last_point) < 0.01 and
            now - last_accept_time < 0.5
        )
        if duplicate or now - last_accept_time < self.min_interval:
            self.dropped_count += 1
            return

        self.last_points[opcode] = (x, y, z)
        self.last_accept_times[opcode] = now
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = 'map'
        point.point.x = x
        point.point.y = y
        point.point.z = z
        pub = self.goal_pub if opcode == OP_FINAL_GOAL else self.waypoint_pub
        pub.publish(point)
        self.published_count += 1
        self.get_logger().info(
            '%s accepted on Go2: x=%.3f y=%.3f z=%.3f' %
            (channel, x, y, z))

    def report_status(self):
        with self.client_lock:
            connected = self.client is not None
        self.get_logger().info(
            'laptop=%s safety=%s received=%d published=%d dropped=%d '
            'ready_rejected=%d' %
            ('connected' if connected else 'waiting',
             'ready' if self._commands_ready_now() else 'blocked',
             self.received_count, self.published_count, self.dropped_count,
             self.ready_rejected_count))

    def close(self):
        self.closed.set()
        try:
            self.server.close()
        except OSError:
            pass
        with self.client_lock:
            conn = self.client
            self.client = None
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
                conn.close()
            except OSError:
                pass


def enabled_from_env():
    value = os.environ.get('FAR_COMMAND_ENABLE', '1').strip().lower()
    return value not in ('0', 'false', 'off', 'no')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int,
                        default=int(os.environ.get('FAR_COMMAND_PORT', '8899')))
    options, ros_args = parser.parse_known_args(args)
    if not enabled_from_env():
        print('waypoint command channel disabled by FAR_COMMAND_ENABLE')
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
