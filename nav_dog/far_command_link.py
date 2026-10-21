#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LEGACY / NOT STARTED: old Go2-side FAR-to-waypoint arbitration.

The final architecture runs FAR on the laptop and publishes the sole
/way_point from global_path_receiver.py. This file is retained only for
history and must not be synchronized or launched by production scripts.
"""

import argparse
import math
import os
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from far_command_protocol import (
    FRAME_HEADER, MAX_FRAME_BYTES, OP_FINAL_GOAL, decode_command_payload,
)


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z +
               quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z))


def sensor_pose_to_body_pose(sensor_x, sensor_y, sensor_z, yaw,
                             sensor_offset_x, sensor_offset_y):
    """Match localPlanner's sensor-to-body convention.

    The hold target and localPlanner's vehicle pose use the same algebra, so
    their difference is zero while both sides receive the same offset. Tests
    lock this invariant to prevent the no-goal target from becoming a moving
    "carrot" ahead of the robot.
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    body_x = (sensor_x - cos_yaw * sensor_offset_x +
              sin_yaw * sensor_offset_y)
    body_y = (sensor_y - sin_yaw * sensor_offset_x -
              cos_yaw * sensor_offset_y)
    return body_x, body_y, sensor_z


class FarCommandLink(Node):

    def __init__(self, port, sensor_offset_x=0.16143,
                 sensor_offset_y=0.0):
        super().__init__('far_command_link')
        self.port = int(port)
        self.sensor_offset_x = float(sensor_offset_x)
        self.sensor_offset_y = float(sensor_offset_y)
        self.closed = threading.Event()
        self.client = None
        self.client_lock = threading.Lock()
        self.latest_odom = None
        self.goal_received = False
        self.far_waypoint_seen = False
        self.last_goal = None
        self.last_goal_time = 0.0
        self.last_far_waypoint = None
        self.last_far_waypoint_time = 0.0
        self.min_interval = 0.2
        self.received_count = 0
        self.goal_publish_count = 0
        self.far_waypoint_count = 0
        self.ignored_far_waypoint_count = 0
        self.dropped_count = 0
        self.hold_publish_count = 0

        self.goal_pub = self.create_publisher(PointStamped, '/goal_point', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.create_subscription(
            Odometry, '/lio_sam_ros2/mapping/re_location_odometry',
            self.odom_callback, 10)
        self.create_subscription(
            PointStamped, '/far/way_point', self.far_waypoint_callback, 10)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

        self.acceptor = threading.Thread(target=self.accept_loop, daemon=True)
        self.acceptor.start()
        self.create_timer(0.1, self.publish_hold_waypoint)
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'final-goal channel listening on %d; /far/way_point -> /way_point; '
            'holding relocalized body pose (sensor offset x=%.3f y=%.3f) '
            'until FAR produces its first waypoint' %
            (self.port, self.sensor_offset_x, self.sensor_offset_y))

    def odom_callback(self, message):
        position = message.pose.pose.position
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        self.latest_odom = (position.x, position.y, position.z, yaw)
        self.publish_hold_waypoint()

    def publish_waypoint(self, point, publisher):
        stamp = self.get_clock().now().to_msg()
        point.header.stamp = stamp
        point.header.frame_id = 'map'
        publisher.publish(point)

    def publish_hold_waypoint(self):
        """Keep localPlanner at the current pose before FAR has a waypoint.

        Upstream autonomy mode defaults its target to (0, 0). FAR also publishes
        a graph-reset stop point before any goal is requested. Both must be kept
        away from localPlanner until a real FAR waypoint follows an operator
        goal. This is not a motion gate: once FAR publishes that waypoint, the
        hold is permanently disabled for the process lifetime.
        """
        if self.closed.is_set() or self.far_waypoint_seen:
            return

        if self.latest_odom is None:
            return

        sensor_x, sensor_y, sensor_z, yaw = self.latest_odom
        x, y, z = sensor_pose_to_body_pose(
            sensor_x, sensor_y, sensor_z, yaw,
            self.sensor_offset_x, self.sensor_offset_y)

        point = PointStamped()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.publish_waypoint(point, self.waypoint_pub)
        self.hold_publish_count += 1

    def far_waypoint_callback(self, message):
        if not self.goal_received:
            self.ignored_far_waypoint_count += 1
            if self.ignored_far_waypoint_count == 1:
                self.get_logger().info(
                    'ignoring FAR graph-reset waypoint until an operator goal arrives')
            return
        x = float(message.point.x)
        y = float(message.point.y)
        z = float(message.point.z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            self.dropped_count += 1
            self.get_logger().warn('rejected non-finite FAR waypoint')
            return
        now = time.monotonic()
        duplicate = (
            self.last_far_waypoint is not None and
            math.dist((x, y, z), self.last_far_waypoint) < 0.01 and
            now - self.last_far_waypoint_time < 0.5
        )
        if duplicate or now - self.last_far_waypoint_time < self.min_interval:
            self.dropped_count += 1
            return

        self.last_far_waypoint = (x, y, z)
        self.last_far_waypoint_time = now
        self.far_waypoint_seen = True
        point = PointStamped()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.publish_waypoint(point, self.waypoint_pub)
        self.far_waypoint_count += 1
        if self.far_waypoint_count == 1:
            self.get_logger().info(
                'first FAR waypoint received; localPlanner hold target disabled')
        self.get_logger().info(
            'FAR waypoint forwarded on Go2: x=%.3f y=%.3f z=%.3f' % (x, y, z))

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
        if opcode != OP_FINAL_GOAL:
            self.dropped_count += 1
            self.get_logger().warn('rejected non-goal command opcode %d' % opcode)
            return
        if not all(math.isfinite(value) for value in (x, y, z)):
            self.dropped_count += 1
            self.get_logger().warn('rejected non-finite final goal')
            return

        now = time.monotonic()
        duplicate = (
            self.last_goal is not None and
            math.dist((x, y, z), self.last_goal) < 0.01 and
            now - self.last_goal_time < 0.5
        )
        if duplicate or now - self.last_goal_time < self.min_interval:
            self.dropped_count += 1
            return

        self.last_goal = (x, y, z)
        self.last_goal_time = now
        self.goal_received = True
        point = PointStamped()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.publish_waypoint(point, self.goal_pub)
        self.goal_publish_count += 1
        self.get_logger().info(
            'final goal accepted on Go2: x=%.3f y=%.3f z=%.3f' % (x, y, z))

    def report_status(self):
        with self.client_lock:
            connected = self.client is not None
        self.get_logger().info(
            'laptop=%s received=%d goals=%d far_waypoints=%d ignored_far=%d '
            'dropped=%d hold=%s/%d' %
            ('connected' if connected else 'waiting', self.received_count,
             self.goal_publish_count, self.far_waypoint_count,
             self.ignored_far_waypoint_count, self.dropped_count,
             'off' if self.far_waypoint_seen else 'on',
             self.hold_publish_count))

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
    parser.add_argument(
        '--sensor-offset-x', type=float,
        default=float(os.environ.get('FAR_SENSOR_OFFSET_X', '0.16143')))
    parser.add_argument(
        '--sensor-offset-y', type=float,
        default=float(os.environ.get('FAR_SENSOR_OFFSET_Y', '0.0')))
    options, ros_args = parser.parse_known_args(args)
    if not enabled_from_env():
        print('waypoint command channel disabled by FAR_COMMAND_ENABLE')
        return 0
    rclpy.init(args=ros_args)
    node = FarCommandLink(
        options.port, options.sensor_offset_x, options.sensor_offset_y)
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
