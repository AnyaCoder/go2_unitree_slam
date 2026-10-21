#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Own the Go2-side ``/way_point`` from the laptop's global path.

The laptop plans globally on the fixed map and sends an ordered map-frame
polyline. This node tracks the robot along that polyline and publishes one
look-ahead target to localPlanner. It is the sole public ``/way_point``
publisher. A missing or stale path holds localPlanner at the current body pose;
an old path is never allowed to keep pulling the robot forward.
"""

import argparse
import math
import os
import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from nav_path_protocol import (
    FRAME_HEADER, MAX_FRAME_BYTES, OP_GLOBAL_PATH, decode_command_payload,
)

PATH_STALE_SECONDS = 2.0
LOOKAHEAD_DISTANCE = 1.2
CORNER_ANGLE = math.radians(35.0)
MIN_CORNER_DISTANCE = 0.15


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z +
               quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z))


def sensor_pose_to_body_pose(sensor_x, sensor_y, sensor_z, yaw,
                             sensor_offset_x, sensor_offset_y):
    """Match localPlanner's sensor-to-body coordinate convention."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    body_x = (sensor_x - cos_yaw * sensor_offset_x +
              sin_yaw * sensor_offset_y)
    body_y = (sensor_y - sin_yaw * sensor_offset_x -
              cos_yaw * sensor_offset_y)
    return body_x, body_y, sensor_z


def _interpolate(first, second, distance):
    span = math.dist(first, second)
    if span <= 0.0:
        return second
    ratio = max(0.0, min(1.0, distance / span))
    return tuple(first[axis] + ratio * (second[axis] - first[axis])
                 for axis in range(3))


def _turn_angle(previous, corner, following):
    first_x = corner[0] - previous[0]
    first_y = corner[1] - previous[1]
    second_x = following[0] - corner[0]
    second_y = following[1] - corner[1]
    first_norm = math.hypot(first_x, first_y)
    second_norm = math.hypot(second_x, second_y)
    if first_norm <= 0.0 or second_norm <= 0.0:
        return 0.0
    cosine = (first_x * second_x + first_y * second_y) / (
        first_norm * second_norm)
    return math.acos(max(-1.0, min(1.0, cosine)))


def choose_path_target(points, body_pose, lookahead=LOOKAHEAD_DISTANCE,
                       corner_angle=CORNER_ANGLE,
                       min_corner_distance=MIN_CORNER_DISTANCE):
    """Choose a forward target on an ordered map-frame polyline.

    Progress is measured from the nearest segment projection. The target stops
    at a sharp vertex when that vertex is close enough, preventing a large
    look-ahead point from cutting through the inside of a corner.
    """
    if not points:
        return None
    if len(points) == 1:
        return tuple(points[0])

    body_x, body_y = body_pose[0], body_pose[1]
    nearest = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            ratio = 0.0
        else:
            ratio = ((body_x - first[0]) * dx +
                     (body_y - first[1]) * dy) / length_sq
            ratio = max(0.0, min(1.0, ratio))
        projected = (
            first[0] + ratio * dx, first[1] + ratio * dy,
            first[2] + ratio * (second[2] - first[2]))
        distance_sq = ((body_x - projected[0]) ** 2 +
                       (body_y - projected[1]) ** 2)
        if nearest is None or distance_sq < nearest[0]:
            nearest = (distance_sq, index, projected)

    _, segment_index, cursor = nearest
    best = None
    travelled = 0.0
    for index in range(segment_index, len(points) - 1):
        first = points[index]
        second = points[index + 1]
        if index == segment_index:
            first = cursor
        segment_length = math.dist(first, second)
        if segment_length <= 0.0:
            continue
        if travelled + segment_length >= lookahead:
            return _interpolate(first, second, lookahead - travelled)
        best = tuple(second)
        travelled += segment_length
        if index + 2 < len(points):
            corner = points[index + 1]
            angle = _turn_angle(first if index > segment_index else points[index],
                                corner, points[index + 2])
            if angle >= corner_angle and travelled >= min_corner_distance:
                return tuple(corner)
    return best if best is not None else tuple(points[-1])


class GlobalPathReceiver(Node):
    def __init__(self, port, sensor_offset_x=0.16143,
                 sensor_offset_y=0.0, path_stale_seconds=PATH_STALE_SECONDS):
        super().__init__('global_path_receiver')
        self.port = int(port)
        self.sensor_offset_x = float(sensor_offset_x)
        self.sensor_offset_y = float(sensor_offset_y)
        self.path_stale_seconds = float(path_stale_seconds)
        self.closed = threading.Event()
        self.client = None
        self.client_lock = threading.Lock()
        self.path_lock = threading.Lock()
        self.latest_odom = None
        self.latest_path = ()
        self.path_updated_at = 0.0
        self.path_sequence = None
        self.current_target = None
        self.received_count = 0
        self.accepted_count = 0
        self.dropped_count = 0
        self.hold_publish_count = 0
        self.target_publish_count = 0
        self.stale_reported = False
        self.waiting_for_odom_reported = False

        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.create_subscription(
            Odometry, '/lio_sam_ros2/mapping/re_location_odometry',
            self.odom_callback, 10)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

        self.acceptor = threading.Thread(target=self.accept_loop, daemon=True)
        self.acceptor.start()
        self.create_timer(0.1, self.publish_target)
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'global-path receiver listening on %d; sole /way_point publisher; '
            'lookahead=%.2fm stale=%.1fs sensor offset x=%.5f y=%.5f' %
            (self.port, LOOKAHEAD_DISTANCE, self.path_stale_seconds,
             self.sensor_offset_x, self.sensor_offset_y))

    def odom_callback(self, message):
        position = message.pose.pose.position
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        self.latest_odom = (position.x, position.y, position.z, yaw)

    def publish_waypoint(self, point, hold):
        if self.latest_odom is None:
            if not self.waiting_for_odom_reported:
                self.get_logger().info('waiting for relocalized odometry before /way_point')
                self.waiting_for_odom_reported = True
            return
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.point.x = float(point[0])
        message.point.y = float(point[1])
        message.point.z = float(point[2])
        self.waypoint_pub.publish(message)
        if hold:
            self.hold_publish_count += 1
        else:
            self.target_publish_count += 1

    def body_pose(self):
        sensor_x, sensor_y, sensor_z, yaw = self.latest_odom
        return sensor_pose_to_body_pose(
            sensor_x, sensor_y, sensor_z, yaw,
            self.sensor_offset_x, self.sensor_offset_y)

    def publish_target(self):
        if self.latest_odom is None:
            return
        body = self.body_pose()
        with self.path_lock:
            path = self.latest_path
            updated_at = self.path_updated_at
        fresh = bool(path) and (
            time.monotonic() - updated_at <= self.path_stale_seconds)
        if not fresh:
            if path and not self.stale_reported:
                self.get_logger().warn(
                    'global path is stale or empty; holding current body pose')
                self.stale_reported = True
            self.current_target = body
            self.publish_waypoint(body, hold=True)
            return
        self.stale_reported = False
        try:
            target = choose_path_target(path, body)
        except (IndexError, TypeError, ValueError) as error:
            self.get_logger().warn('invalid global path tracking state: %s' % error)
            target = None
        if target is None:
            self.current_target = body
            self.publish_waypoint(body, hold=True)
            return
        self.current_target = target
        self.publish_waypoint(target, hold=False)

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
                'laptop global-path connected from %s:%d' % address)
            threading.Thread(
                target=self.reader_loop, args=(connection,), daemon=True).start()

    def reader_loop(self, connection):
        buffer = bytearray()
        try:
            while not self.closed.is_set():
                data = connection.recv(8192)
                if not data:
                    break
                buffer.extend(data)
                while len(buffer) >= FRAME_HEADER.size:
                    length = FRAME_HEADER.unpack_from(buffer)[0]
                    if length < 1 or length > MAX_FRAME_BYTES:
                        self.dropped_count += 1
                        self.get_logger().warn(
                            'invalid global-path frame length %d; disconnecting' %
                            length)
                        return
                    end = FRAME_HEADER.size + length
                    if len(buffer) < end:
                        break
                    payload = bytes(buffer[FRAME_HEADER.size:end])
                    del buffer[:end]
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
            self.get_logger().info('laptop global-path disconnected')

    def handle_payload(self, payload):
        self.received_count += 1
        try:
            opcode, (sequence, points) = decode_command_payload(payload)
        except (ValueError, TypeError) as error:
            self.dropped_count += 1
            self.get_logger().warn('rejected global path: %s' % error)
            return
        if opcode != OP_GLOBAL_PATH:
            self.dropped_count += 1
            return
        path = tuple(points)
        if 0 < len(path) < 2:
            self.dropped_count += 1
            self.get_logger().warn('rejected one-point global path; holding body pose')
            with self.path_lock:
                self.latest_path = ()
                self.path_sequence = sequence
                self.path_updated_at = time.monotonic()
            return
        with self.path_lock:
            changed = path != self.latest_path
            self.latest_path = path
            self.path_sequence = sequence
            self.path_updated_at = time.monotonic()
        self.accepted_count += 1
        if changed:
            if path:
                self.get_logger().info(
                    'global path accepted: sequence=%d points=%d' %
                    (sequence, len(path)))
            else:
                self.get_logger().info(
                    'global path cleared: sequence=%d; holding body pose' %
                    sequence)

    def report_status(self):
        with self.client_lock:
            connected = self.client is not None
        with self.path_lock:
            path_points = len(self.latest_path)
            sequence = self.path_sequence
            age = time.monotonic() - self.path_updated_at
        self.get_logger().info(
            'laptop=%s received=%d accepted=%d dropped=%d sequence=%s '
            'path_points=%d age=%.1fs target=%s hold=%d command=%d' %
            ('connected' if connected else 'waiting', self.received_count,
             self.accepted_count, self.dropped_count, sequence, path_points,
             age, 'none' if self.current_target is None else
             '(%.2f, %.2f, %.2f)' % self.current_target,
             self.hold_publish_count, self.target_publish_count))

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
    parser.add_argument(
        '--port', type=int,
        default=int(os.environ.get('FAR_COMMAND_PORT', '8899')))
    parser.add_argument(
        '--sensor-offset-x', type=float,
        default=float(os.environ.get('FAR_SENSOR_OFFSET_X', '0.16143')))
    parser.add_argument(
        '--sensor-offset-y', type=float,
        default=float(os.environ.get('FAR_SENSOR_OFFSET_Y', '0.0')))
    parser.add_argument('--path-stale-seconds', type=float,
                        default=PATH_STALE_SECONDS)
    options, ros_args = parser.parse_known_args(args)
    enabled = os.environ.get('GO2_WAYPOINT_ENABLE', '1').strip().lower()
    if enabled in ('0', 'false', 'off', 'no'):
        print('global-path receiver disabled')
        return 0
    rclpy.init(args=ros_args)
    node = GlobalPathReceiver(
        options.port, options.sensor_offset_x, options.sensor_offset_y,
        options.path_stale_seconds)
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
