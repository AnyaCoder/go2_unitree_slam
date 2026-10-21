#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Send the laptop FAR global path to the Go2 over the low-bandwidth link.

The laptop owns global planning. FAR's ordered ``/viz_path_topic`` marker is
reduced to an ordered map-frame polyline and sent on TCP 8899. Point clouds,
TF, robot state and velocity commands never use this channel.
"""

import argparse
import math
import os
import socket
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'nav_dog'))
from nav_path_protocol import (  # noqa: E402
    MAX_PATH_POINTS, pack_global_path_frame,
)

MIN_POINT_SPACING = 0.02
RESEND_INTERVAL = 0.5
PATH_SOURCE_TIMEOUT = 2.0
STATE_TIMEOUT = 1.0
PATH_FRAME = 'map'


def select_path_points(points, max_points=MAX_PATH_POINTS,
                       min_spacing=MIN_POINT_SPACING):
    """Validate, remove duplicate points and keep the ordered polyline."""
    clean = []
    for point in points:
        values = (float(point.x), float(point.y), float(point.z))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('global path contains a non-finite point')
        if clean:
            previous = clean[-1]
            if math.dist(values, previous) < min_spacing:
                continue
        clean.append(values)
    if len(clean) <= max_points:
        return clean

    # Preserve the endpoints and spread the remaining budget over path arc
    # length instead of keeping an arbitrary prefix.
    lengths = [0.0]
    for first, second in zip(clean, clean[1:]):
        lengths.append(lengths[-1] + math.dist(first, second))
    total = lengths[-1]
    if total <= 0.0:
        return clean[:1]
    selected = []
    cursor = 0
    for index in range(max_points):
        distance = total * index / float(max_points - 1)
        while cursor + 1 < len(lengths) and lengths[cursor + 1] < distance:
            cursor += 1
        if cursor + 1 >= len(lengths):
            point = clean[-1]
        else:
            span = lengths[cursor + 1] - lengths[cursor]
            ratio = 0.0 if span <= 0.0 else (
                (distance - lengths[cursor]) / span)
            first, second = clean[cursor], clean[cursor + 1]
            point = tuple(
                first[axis] + ratio * (second[axis] - first[axis])
                for axis in range(3))
        if not selected or point != selected[-1]:
            selected.append(point)
    selected[-1] = clean[-1]
    return selected


class GlobalPathSender(Node):
    def __init__(self, host, port):
        super().__init__('global_path_sender')
        self.host = host
        self.port = int(port)
        self.closed = threading.Event()
        self.connection_lock = threading.Lock()
        self.path_lock = threading.Lock()
        self.connection = None
        self.latest_path = ()
        self.path_version = 0
        self.path_updated_at = 0.0
        self.state_updated_at = 0.0
        self.sequence = 0
        self.sent_count = 0
        self.sent_path_count = 0
        self.dropped_count = 0
        self.rejected_count = 0
        self.last_warning = None

        self.create_subscription(
            Marker, '/viz_path_topic', self.on_path, 5)
        self.create_subscription(
            Odometry, '/state_estimation', self.on_state, 10)
        self.create_timer(5.0, self.report_status)
        self.sender = threading.Thread(target=self.send_loop, daemon=True)
        self.sender.start()
        self.get_logger().info(
            'global-path sender target %s:%d; waiting for laptop FAR '
            '/viz_path_topic' % (self.host, self.port))

    def on_state(self, message):
        self.state_updated_at = time.monotonic()

    def on_path(self, message):
        if message.ns and message.ns != 'global_path':
            return
        received_at = time.monotonic()
        if message.action == Marker.DELETE or not message.points:
            path = ()
        else:
            frame = message.header.frame_id.strip('/')
            if frame and frame != PATH_FRAME:
                self.rejected_count += 1
                warning = 'global path frame %s is not %s' % (frame, PATH_FRAME)
                if warning != self.last_warning:
                    self.get_logger().warn(warning)
                    self.last_warning = warning
                return
            try:
                path = tuple(select_path_points(message.points))
            except ValueError as error:
                self.rejected_count += 1
                self.get_logger().warn('rejected laptop FAR path: %s' % error)
                path = ()
            if 0 < len(path) < 2:
                path = ()
        with self.path_lock:
            changed = path != self.latest_path
            self.latest_path = path
            self.path_updated_at = received_at
            if changed:
                self.path_version += 1
        self.last_warning = None
        if changed:
            if path:
                self.get_logger().info(
                    'laptop FAR path updated: %d points; queued for Go2' %
                    len(path))
            else:
                self.get_logger().info(
                    'laptop FAR path cleared; queued stop hold')

    def snapshot(self):
        with self.path_lock:
            return (self.latest_path, self.path_version,
                    self.path_updated_at, self.state_updated_at)

    def connect(self):
        try:
            connection = socket.create_connection(
                (self.host, self.port), timeout=3.0)
        except OSError:
            return None
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self.connection_lock:
            self.connection = connection
        self.get_logger().info(
            'connected to Go2 global-path channel %s:%d' %
            (self.host, self.port))
        return connection

    def disconnect(self, connection):
        with self.connection_lock:
            if self.connection is connection:
                self.connection = None
        try:
            connection.close()
        except OSError:
            pass
        self.get_logger().info('Go2 global-path channel disconnected')

    def send_loop(self):
        last_freshness_key = None
        last_fresh = None
        last_send_time = 0.0
        while not self.closed.is_set():
            path, version, path_time, state_time = self.snapshot()
            now = time.monotonic()
            state_age = now - state_time if state_time > 0.0 else float('inf')
            path_age = now - path_time if path_time > 0.0 else float('inf')
            fresh = bool(path) and state_age <= STATE_TIMEOUT and (
                path_age <= PATH_SOURCE_TIMEOUT)
            send_path = path if fresh else ()
            freshness_key = (version, fresh)
            due = (freshness_key != last_freshness_key or
                   now - last_send_time >= RESEND_INTERVAL)
            if not due:
                self.closed.wait(0.05)
                continue
            with self.connection_lock:
                connection = self.connection
            if connection is None:
                connection = self.connect()
                if connection is None:
                    self.closed.wait(1.0)
                    continue
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            try:
                frame = pack_global_path_frame(send_path, self.sequence)
                connection.sendall(frame)
            except (OSError, ValueError) as error:
                self.dropped_count += 1
                self.get_logger().warn('global-path send failed: %s' % error)
                self.disconnect(connection)
                self.closed.wait(0.5)
                continue
            self.sent_count += 1
            if last_freshness_key is None or freshness_key[0] != last_freshness_key[0]:
                self.sent_path_count += 1
            if path and last_fresh is not None and last_fresh != fresh:
                if fresh:
                    self.get_logger().info(
                        'global path is fresh again; releasing Go2 hold')
                else:
                    self.get_logger().warn(
                        'holding Go2: stale FAR path or robot state '
                        '(path_age=%.1fs state_age=%.1fs)' %
                        (path_age, state_age))
            last_fresh = fresh
            last_freshness_key = freshness_key
            last_send_time = now

    def report_status(self):
        with self.connection_lock:
            connected = self.connection is not None
        path, version, path_time, state_time = self.snapshot()
        now = time.monotonic()
        path_age = now - path_time if path_time > 0.0 else float('inf')
        state_age = now - state_time if state_time > 0.0 else float('inf')
        self.get_logger().info(
            'target=%s:%d %s path_points=%d updates=%d sent=%d '
            'path_frames=%d dropped=%d rejected=%d '
            'path_age=%.1fs state_age=%.1fs' %
            (self.host, self.port,
             'connected' if connected else 'reconnecting', len(path), version,
             self.sent_count, self.sent_path_count, self.dropped_count,
             self.rejected_count, path_age, state_age))

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
    enabled = os.environ.get('GO2_WAYPOINT_ENABLE', '1').strip().lower()
    if options.disable or enabled in ('0', 'false', 'off', 'no'):
        print('global-path sender disabled')
        return 0
    rclpy.init(args=ros_args)
    node = GlobalPathSender(options.host, options.port)
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
