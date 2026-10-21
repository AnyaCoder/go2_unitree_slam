#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Start Go2 relocation from RViz, then publish the selected /initialpose."""

import argparse
import math
import socket
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String
from unitree_interfaces.msg import QtCommand

from initialpose_protocol import (
    FRAME_HEADER, MAX_FRAME_BYTES, decode_initial_pose_payload,
)


class InitialPoseLink(Node):

    def __init__(self, port):
        super().__init__('initialpose_link')
        self.port = int(port)
        self.closed = threading.Event()
        self.client = None
        self.client_lock = threading.Lock()
        self.received_count = 0
        self.published_count = 0
        self.dropped_count = 0
        self.last_accept_time = 0.0
        self.relocation_count = 0
        self.pending_timer = None
        self.pending_pose = None
        self.relocation_waiting = False
        self.command_index = 9000

        self.command_publisher = self.create_publisher(
            QtCommand, '/qt_command', 10)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Log, '/rosout', self.rosout_callback, 50)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)
        threading.Thread(target=self.accept_loop, daemon=True).start()
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'initial-pose channel listening on %d -> /qt_command(6) + /initialpose; '
            'cold start waits for globalLocalize map initialization' % self.port)

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
            self.get_logger().info(
                'laptop initial-pose connected from %s:%d' % address)
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
                            'invalid initial-pose frame length %d' % length)
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
            self.get_logger().info('laptop initial-pose disconnected')

    def handle_payload(self, payload):
        self.received_count += 1
        try:
            x, y, yaw = decode_initial_pose_payload(payload)
        except ValueError as error:
            self.dropped_count += 1
            self.get_logger().warn('rejected initial pose: %s' % error)
            return
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.dropped_count += 1
            self.get_logger().warn('rejected non-finite initial pose')
            return
        now = time.monotonic()
        if now - self.last_accept_time < 0.2:
            self.dropped_count += 1
            return
        self.last_accept_time = now
        relocation_running = any(
            name.lstrip('/') == 'lio_sam_ros2_globalLocalize'
            for name in self.get_node_names())
        if relocation_running:
            self.get_logger().info(
                'relocation node already running; applying pose directly')
            self.schedule_pose(x, y, yaw, 0.2)
            return

        self.pending_pose = (x, y, yaw)
        if not self.relocation_waiting:
            self.relocation_waiting = True
            self.start_relocation()
            self.get_logger().info(
                'relocation started; waiting for globalLocalize map initialization')
        else:
            self.get_logger().info(
                'relocation already starting; updated pending initial pose')
        # This is only a fail-safe. The normal path is triggered by the
        # globalLocalize readiness log below; first-map loading routinely takes
        # longer than five seconds on Go2.
        self.schedule_pose(x, y, yaw, 60.0)

    def schedule_pose(self, x, y, yaw, delay):
        if self.pending_timer is not None:
            self.pending_timer.cancel()
        self.pending_timer = threading.Timer(
            delay, self.publish_initial_pose, args=(x, y, yaw))
        self.pending_timer.daemon = True
        self.pending_timer.start()

    def rosout_callback(self, message):
        if not self.relocation_waiting or self.pending_pose is None:
            return
        # Foxy often reports the actual source as ``rclcpp`` instead of the
        # node name in Log.name.  The readiness message text is specific
        # enough here; keep the node-name fast path for distros that provide it.
        logger_is_global_localize = message.name.endswith(
            'lio_sam_ros2_globalLocalize')
        logger_is_rclcpp = message.name == 'rclcpp'
        if not (logger_is_global_localize or logger_is_rclcpp):
            return
        if 'Map Optimization Started' not in message.msg:
            return
        x, y, yaw = self.pending_pose
        self.pending_pose = None
        self.relocation_waiting = False
        self.get_logger().info(
            'globalLocalize map ready; submitting pending initial pose')
        self.schedule_pose(x, y, yaw, 0.2)

    def start_relocation(self):
        self.command_index += 1
        if self.command_index > 9999:
            self.command_index = 9001
        command = QtCommand()
        command.command = 6
        command.attribute = 2
        command.seq = String(data='index:%d;' % self.command_index)
        self.command_publisher.publish(command)
        self.relocation_count += 1
        self.get_logger().info(
            'sent Go2 start-relocation command index=%d' % self.command_index)
        return True

    def publish_initial_pose(self, x, y, yaw):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = float(np.sin(yaw * 0.5))
        message.pose.pose.orientation.w = float(np.cos(yaw * 0.5))
        message.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        self.publisher.publish(message)
        self.published_count += 1
        self.get_logger().info(
            'published Go2 /initialpose: x=%.3f y=%.3f yaw=%.1f deg' %
            (x, y, math.degrees(yaw)))

    def report_status(self):
        with self.client_lock:
            connected = self.client is not None
        self.get_logger().info(
            'laptop=%s received=%d relocation=%d published=%d dropped=%d' %
            ('connected' if connected else 'waiting', self.received_count,
             self.relocation_count, self.published_count, self.dropped_count))

    def close(self):
        self.closed.set()
        if self.pending_timer is not None:
            self.pending_timer.cancel()
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
    parser.add_argument('--port', type=int, default=8900)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = InitialPoseLink(options.port)
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
