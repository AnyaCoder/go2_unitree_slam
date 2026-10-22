#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stream Go2 state and native FAR visualization to the laptop over TCP 8898.

No cloud topics are subscribed. The link is an uplink server: the laptop
connects, and the Go2 sends only state, joint data, FAR markers, the active path
and the current FAR waypoint.
"""

import argparse
import socket
import threading
import time

import rclpy
from rclpy.node import Node

from far_viz_protocol import MAX_FRAME_BYTES, TOPICS, encode_topic, qos_for


# Dynamic messages are represented by their newest value only. This prevents a
# slow 2.4 GHz link from replaying an old pose/path after a short Wi-Fi stall.
DYNAMIC_TOPICS = frozenset((
    '/nav/state_estimation', '/tf', '/tf_static', '/joint_states',
    '/path', '/way_point', '/navigation_boundary',
))


class LatestFrameQueue:
    """Thread-safe, per-topic latest-value queue with fair topic selection."""

    def __init__(self, topic_count):
        self._condition = threading.Condition()
        self._frames = [None] * topic_count
        self._sequence = [0] * topic_count
        self._pending = set()
        self._next_sequence = 0
        self._closed = False
        self.coalesced = 0

    def put(self, index, frame):
        with self._condition:
            if self._closed:
                return
            if self._frames[index] is not None:
                self.coalesced += 1
            self._next_sequence += 1
            self._sequence[index] = self._next_sequence
            self._frames[index] = frame
            self._pending.add(index)
            self._condition.notify()

    def get(self, timeout):
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self._pending and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            if not self._pending:
                return None
            index = min(self._pending, key=self._sequence.__getitem__)
            self._pending.remove(index)
            frame = self._frames[index]
            self._frames[index] = None
            return index, frame

    def clear(self):
        with self._condition:
            for index in self._pending:
                self._frames[index] = None
            self._pending.clear()

    @property
    def pending(self):
        with self._condition:
            return len(self._pending)

    def close(self):
        with self._condition:
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()


class FarVizLink(Node):

    def __init__(self, port):
        super().__init__('far_viz_link')
        self.port = int(port)
        self.client = None
        self.client_lock = threading.Lock()
        self.outbound = LatestFrameQueue(len(TOPICS))
        self.closed = threading.Event()
        self.last_payload = [None] * len(TOPICS)
        self.last_callback_time = [0.0] * len(TOPICS)
        self.topic_counts = [0] * len(TOPICS)
        self.sent_bytes = 0

        self.subscribers = []
        for index, spec in enumerate(TOPICS):
            self.subscribers.append(self.create_subscription(
                spec.msg_type, spec.remote_topic,
                lambda msg, i=index: self.topic_callback(i, msg),
                qos_for(spec)))

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
            'state/FAR visualization uplink listening on %d; clouds disabled' %
            self.port)

    def connected(self):
        with self.client_lock:
            return self.client is not None

    def topic_callback(self, index, message):
        if not self.connected():
            return
        spec = TOPICS[index]
        now = time.monotonic()
        if now - self.last_callback_time[index] < 1.0 / spec.hz:
            return
        self.last_callback_time[index] = now

        payload = encode_topic(index, message)
        if len(payload) > MAX_FRAME_BYTES:
            self.get_logger().warn(
                'drop oversized %s frame: %d bytes' %
                (spec.remote_topic, len(payload)))
            return
        # Identical FAR marker payloads need not consume the slow uplink.
        # Dynamic state always advances, including while the robot is still.
        if (spec.remote_topic not in DYNAMIC_TOPICS
                and payload == self.last_payload[index]):
            return
        if not self.connected():
            return
        self.last_payload[index] = payload
        self.topic_counts[index] += 1
        self.outbound.put(index, payload)

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
            self.outbound.clear()
            self.last_payload = [None] * len(TOPICS)
            self.get_logger().info('laptop state connected from %s:%d' % address)
            threading.Thread(
                target=self.reader_loop, args=(connection,), daemon=True).start()

    def writer_loop(self):
        while not self.closed.is_set():
            item = self.outbound.get(timeout=0.5)
            if item is None:
                continue
            index, frame = item
            with self.client_lock:
                connection = self.client
            if connection is None:
                continue
            try:
                connection.sendall(frame)
                self.sent_bytes += len(frame)
            except OSError:
                self.disconnect(connection)

    def reader_loop(self, connection):
        """The uplink is one-way; discard any inbound bytes."""
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
        queued = self.outbound.pending
        def count(topic):
            return next((self.topic_counts[i] for i, spec in enumerate(TOPICS)
                         if spec.remote_topic == topic), 0)
        self.get_logger().info(
            'laptop=%s queue=%d coalesced=%d sent=%.1fKB/s odom=%d '
            'joints=%d path=%d waypoint=%d graph=%d poly=%d boundary=%d' %
            ('connected' if connected else 'waiting', queued,
             self.outbound.coalesced, self.sent_bytes / 5000.0,
             count('/nav/state_estimation'), count('/joint_states'),
             count('/path'), count('/way_point'), count('/viz_graph_topic'),
             count('/viz_poly_topic'), count('/navigation_boundary')))
        self.sent_bytes = 0

    def close(self):
        self.closed.set()
        self.outbound.close()
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
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = FarVizLink(options.port)
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
