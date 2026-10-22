#!/usr/bin/env python3
"""Read-only rate and timestamp audit for the dog-side navigation chain."""

import argparse
import collections
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass, field


@dataclass(frozen=True)
class AuditSpec:
    topic: str
    type_name: str
    min_hz: float = 0.0
    max_hz: float = 0.0
    required: bool = True
    sensor_qos: bool = False
    note: str = ''


@dataclass
class RateWindow:
    arrivals: collections.deque = field(default_factory=lambda: collections.deque(maxlen=1000))
    source_stamps: collections.deque = field(default_factory=lambda: collections.deque(maxlen=1000))
    count: int = 0
    last_arrival: float = 0.0
    last_source_stamp: float = 0.0
    frame_id: str = ''

    def add(self, source_stamp, now, frame_id=''):
        self.count += 1
        self.arrivals.append(float(now))
        self.last_arrival = now
        if source_stamp > 0.0 and math.isfinite(source_stamp):
            self.source_stamps.append(float(source_stamp))
            self.last_source_stamp = float(source_stamp)
        if frame_id:
            self.frame_id = str(frame_id)

    @staticmethod
    def _statistics(values):
        if len(values) < 2:
            return None
        intervals = [b - a for a, b in zip(values, values[1:]) if b > a]
        if not intervals:
            return None
        ordered = sorted(intervals)
        median = statistics.median(intervals)
        p95_index = min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered))) - 1)
        return {
            'hz': 1.0 / median if median > 0.0 else 0.0,
            'median_dt': median,
            'p95_dt': ordered[p95_index],
            'max_dt': ordered[-1],
        }

    def arrival_stats(self):
        return self._statistics(list(self.arrivals))

    def source_stats(self):
        return self._statistics(list(self.source_stamps))

    def age(self, now):
        return now - self.last_arrival if self.last_arrival else float('inf')


DEFAULT_SPECS = (
    AuditSpec('/lio_sam_ros2/mapping/re_location_odometry',
              'nav_msgs/msg/Odometry', 1.0, 8.0,
              note='authoritative global relocalization pose'),
    AuditSpec('/nav/state_estimation', 'nav_msgs/msg/Odometry', 45.0, 60.0,
              note='planner/controller state'),
    AuditSpec('/lio_sam_ros2/deskew/cloud_deskewed',
              'sensor_msgs/msg/PointCloud2', 8.0, 12.0, sensor_qos=True,
              note='deskewed sensor cloud'),
    AuditSpec('/nav/static_cloud_map', 'sensor_msgs/msg/PointCloud2',
              4.5, 6.5, sensor_qos=True, note='fixed-PCD planning cloud'),
    AuditSpec('/terrain_map', 'sensor_msgs/msg/PointCloud2', 4.5, 6.5,
              sensor_qos=True, note='terrain analysis'),
    AuditSpec('/terrain_map_ext', 'sensor_msgs/msg/PointCloud2', 4.5, 6.5,
              sensor_qos=True, note='extended terrain analysis'),
    AuditSpec('/path', 'nav_msgs/msg/Path', 3.0, 8.0, required=False,
              note='localPlanner path; active after Goalpoint'),
    AuditSpec('/way_point', 'geometry_msgs/msg/PointStamped', 0.0, 8.0,
              required=False, note='FAR navigation point; active after Goalpoint'),
    AuditSpec('/goal_point', 'geometry_msgs/msg/PointStamped', 0.0, 10.0,
              required=False, note='operator Goalpoint event'),
    AuditSpec('/cmd_vel', 'geometry_msgs/msg/Twist', 45.0, 60.0,
              required=False, note='pathFollower output'),
    AuditSpec('/stop', 'std_msgs/msg/Int8', 0.0, 10.0,
              required=False, note='state-bridge stop bits'),
    AuditSpec('/api/sport/request', 'unitree_api/msg/Request', 45.0, 60.0,
              required=False, note='sport command; sampled while active'),
)


def extract_header(message):
    header = getattr(message, 'header', None)
    if header is None:
        return ''
    frame_id = getattr(header, 'frame_id', '') or ''
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return str(frame_id)
    source_stamp = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return str(frame_id), source_stamp


def classify(spec, window, now, strict):
    if window.count == 0:
        return ('FAIL' if spec.required and strict else
                'MISSING' if spec.required else 'OPTIONAL')
    stats = window.arrival_stats()
    if stats is None:
        return 'WARMUP'
    hz = stats['hz']
    status = 'PASS'
    if spec.min_hz > 0.0 and hz < spec.min_hz:
        status = 'FAIL' if strict else 'LOW'
    elif spec.max_hz > 0.0 and hz > spec.max_hz:
        status = 'FAIL' if strict else 'HIGH'
    return status


def format_value(value, digits=2):
    if value is None:
        return '-'
    if isinstance(value, float) and not math.isfinite(value):
        return 'inf'
    return ('%.*f' % (digits, value)).rstrip('0').rstrip('.')


def print_report(specs, windows, started, strict):
    now = time.monotonic()
    failures = 0
    print('\n=== Go2 navigation rate audit (%.1f s) ===' % (now - started))
    print('%-46s %8s %8s %8s %8s %-8s %s' %
          ('topic', 'arrive', 'source', 'p95_dt', 'age', 'frame', 'status'))
    for spec in specs:
        window = windows[spec.topic]
        arr = window.arrival_stats()
        src = window.source_stats()
        status = classify(spec, window, now, strict)
        if status == 'FAIL':
            failures += 1
        print('%-46s %8s %8s %8s %8s %-8s %s' % (
            spec.topic,
            format_value(arr['hz'] if arr else None),
            format_value(src['hz'] if src else None),
            format_value(arr['p95_dt'] if arr else None, 4),
            format_value(window.age(now), 2),
            window.frame_id[:8] or '-',
            status,
        ))
        if spec.note:
            print('  %s' % spec.note)
    print('\nunits: arrival/source Hz, p95_dt seconds, age seconds')
    print('configured windows:')
    for spec in specs:
        print('  %-46s %.1f..%.1f Hz%s' % (
            spec.topic, spec.min_hz, spec.max_hz,
            '' if spec.required else ' (optional)'))
    if strict and failures:
        print('\nRESULT: FAIL (%d required topic(s) outside range or missing)' %
              failures)
        return 1
    if strict:
        print('\nRESULT: PASS')
    else:
        print('\nRESULT: report only; use --strict in automation')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--strict', action='store_true')
    parser.add_argument('--topic', action='append', default=[],
                        help='override defaults; repeat as needed')
    parser.add_argument('--json-out', default='')
    options = parser.parse_args(argv)
    if options.duration <= 0.0:
        parser.error('--duration must be positive')

    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        print('ROS 2 Python environment is not sourced: %s' % error, file=sys.stderr)
        return 2

    specs = tuple(DEFAULT_SPECS)
    if options.topic:
        selected = set(options.topic)
        specs = tuple(spec for spec in specs if spec.topic in selected)
        unknown = selected.difference(spec.topic for spec in specs)
        if unknown:
            print('unknown audit topic(s): %s' % ', '.join(sorted(unknown)),
                  file=sys.stderr)
            return 2

    rclpy.init()
    node = rclpy.create_node('go2_nav_rate_audit')
    windows = {spec.topic: RateWindow() for spec in specs}
    subscriptions = []

    def callback(topic, message):
        header = extract_header(message)
        if isinstance(header, tuple):
            frame_id, stamp = header
        else:
            frame_id, stamp = header, 0.0
        windows[topic].add(stamp, time.monotonic(), frame_id)

    try:
        for spec in specs:
            try:
                message_type = get_message(spec.type_name)
            except (ImportError, AttributeError, ModuleNotFoundError) as error:
                print('cannot load %s for %s: %s' %
                      (spec.type_name, spec.topic, error), file=sys.stderr)
                windows[spec.topic].count = 0
                continue
            qos = qos_profile_sensor_data if spec.sensor_qos else 10
            subscriptions.append(node.create_subscription(
                message_type, spec.topic,
                lambda msg, topic=spec.topic: callback(topic, msg), qos))

        started = time.monotonic()
        deadline = started + options.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        result = print_report(specs, windows, started, options.strict)
        if options.json_out:
            payload = {
                'duration_sec': time.monotonic() - started,
                'strict': options.strict,
                'topics': {
                    spec.topic: {
                        'count': windows[spec.topic].count,
                        'arrival': windows[spec.topic].arrival_stats(),
                        'source': windows[spec.topic].source_stats(),
                        'frame_id': windows[spec.topic].frame_id,
                        'status': classify(spec, windows[spec.topic],
                                           time.monotonic(), options.strict),
                    }
                    for spec in specs
                },
            }
            with open(options.json_out, 'w', encoding='utf-8') as stream:
                json.dump(payload, stream, indent=2, sort_keys=True)
        return result
    except KeyboardInterrupt:
        return 130
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
