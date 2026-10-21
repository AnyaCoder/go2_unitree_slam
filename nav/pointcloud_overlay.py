#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Merge laptop-local terrain with the Go2 minimal dynamic obstacle cloud.

The saved PCD is replayed by pcd_map_sim around the real robot odometry.  This
node overlays the low-rate dynamic obstacle cloud without forwarding any raw
robot scan, and publishes the merged map-frame cloud consumed by the unchanged
terrain_analysis and FAR Planner stack.
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def cloud_to_arrays(message):
    """Return finite xyz and intensity arrays from a float32 PointCloud2."""
    count = int(message.width) * int(message.height)
    if count == 0 or message.point_step == 0:
        return None
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        return None
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            return None
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    count = min(count, raw.size // message.point_step)
    if count == 0:
        return None
    rows = raw[:count * message.point_step].reshape(count, message.point_step)
    xyz = np.empty((count, 3), dtype=np.float32)
    for index, name in enumerate(('x', 'y', 'z')):
        offset = int(fields[name].offset)
        xyz[:, index] = rows[:, offset:offset + 4].copy().view(np.float32).ravel()
    if 'intensity' in fields:
        field = fields['intensity']
        if field.datatype == PointField.FLOAT32 and field.count == 1:
            offset = int(field.offset)
            intensity = rows[:, offset:offset + 4].copy().view(np.float32).ravel()
        else:
            intensity = np.zeros(count, dtype=np.float32)
    else:
        intensity = np.zeros(count, dtype=np.float32)
    keep = np.isfinite(xyz).all(axis=1)
    return xyz[keep], intensity[keep]


def make_cloud(points, intensity, frame_id, stamp):
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * len(points)
    data = np.empty((len(points), 4), dtype=np.float32)
    data[:, :3] = points
    data[:, 3] = intensity
    cloud.data = data.tobytes()
    cloud.is_dense = True
    return cloud


class PointcloudOverlay(Node):

    def __init__(self, static_topic, dynamic_topic, output_topic,
                 voxel_size, dynamic_ttl, static_timeout, max_points):
        super().__init__('go2_pointcloud_overlay')
        self.static_topic = static_topic
        self.dynamic_topic = dynamic_topic
        self.output_topic = output_topic
        self.voxel_size = float(voxel_size)
        self.dynamic_ttl = float(dynamic_ttl)
        self.static_timeout = float(static_timeout)
        self.max_points = int(max_points)
        self.static_xyz = None
        self.static_intensity = None
        self.static_stamp = 0.0
        self.dynamic_xyz = None
        self.dynamic_intensity = None
        self.dynamic_stamp = 0.0
        self.static_frames = 0
        self.dynamic_frames = 0
        self.published_frames = 0
        self.dropped_static = 0
        self.last_missing_static_warning = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos)
        self.create_subscription(
            PointCloud2, self.static_topic, self.on_static, qos)
        self.create_subscription(
            PointCloud2, self.dynamic_topic, self.on_dynamic, qos)
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'online scan overlay: %s + %s -> %s' %
            (self.static_topic, self.dynamic_topic, self.output_topic))

    def on_static(self, message):
        if message.header.frame_id not in ('', 'map'):
            self.dropped_static += 1
            return
        arrays = cloud_to_arrays(message)
        if arrays is None:
            self.dropped_static += 1
            return
        self.static_xyz, self.static_intensity = arrays
        self.static_stamp = time.monotonic()
        self.static_frames += 1
        self.publish_merged()

    def on_dynamic(self, message):
        if message.header.frame_id not in ('', 'map'):
            return
        arrays = cloud_to_arrays(message)
        if arrays is None:
            return
        self.dynamic_xyz, self.dynamic_intensity = arrays
        self.dynamic_stamp = time.monotonic()
        self.dynamic_frames += 1
        self.publish_merged()

    def merged_points(self, now):
        if self.static_xyz is None or now - self.static_stamp > self.static_timeout:
            return None
        point_groups = [self.static_xyz]
        intensity_groups = [self.static_intensity]
        if (self.dynamic_xyz is not None and
                now - self.dynamic_stamp <= self.dynamic_ttl):
            point_groups.append(self.dynamic_xyz)
            # Dynamic input is highlighted without affecting terrain geometry.
            intensity_groups.append(np.ones(len(self.dynamic_xyz), dtype=np.float32))
        points = np.concatenate(point_groups, axis=0)
        intensity = np.concatenate(intensity_groups, axis=0)
        if self.voxel_size > 0.0 and len(points):
            keys = np.floor(points / self.voxel_size).astype(np.int64)
            _, keep = np.unique(keys, axis=0, return_index=True)
            points = points[keep]
            intensity = intensity[keep]
        if self.max_points > 0 and len(points) > self.max_points:
            keep = np.linspace(0, len(points) - 1, self.max_points, dtype=np.int64)
            points = points[keep]
            intensity = intensity[keep]
        return points, intensity

    def publish_merged(self):
        now = time.monotonic()
        merged = self.merged_points(now)
        if merged is None:
            if now - self.last_missing_static_warning > 5.0:
                self.get_logger().warn(
                    'waiting for fresh %s; no merged scan published' % self.static_topic)
                self.last_missing_static_warning = now
            return
        points, intensity = merged
        self.publisher.publish(make_cloud(
            points, intensity, 'map', self.get_clock().now().to_msg()))
        self.published_frames += 1

    def report_status(self):
        dynamic_age = (time.monotonic() - self.dynamic_stamp
                       if self.dynamic_stamp > 0.0 else -1.0)
        self.get_logger().info(
            'static=%d dynamic=%d published=%d dynamic_age=%.1fs dropped=%d' %
            (self.static_frames, self.dynamic_frames, self.published_frames,
             dynamic_age, self.dropped_static))


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--static-topic', default='/sim/registered_scan')
    parser.add_argument('--dynamic-topic', default='/go2/min_obstacle_cloud')
    parser.add_argument('--output-topic', default='/registered_scan')
    parser.add_argument('--voxel-size', type=float, default=0.0)
    parser.add_argument('--dynamic-ttl', type=float, default=2.5)
    parser.add_argument('--static-timeout', type=float, default=1.0)
    parser.add_argument('--max-points', type=int, default=200000)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = PointcloudOverlay(
        options.static_topic, options.dynamic_topic, options.output_topic,
        options.voxel_size, options.dynamic_ttl, options.static_timeout,
        options.max_points)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
