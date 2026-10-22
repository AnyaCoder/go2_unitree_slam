#!/usr/bin/env python3
"""Replay the saved navigation PCD as the map_sim-style terrain source.

The cloud geometry uses one deterministic 0.08 m voxel downsample at startup,
then a 6 m x 6 m map-frame window around the current robot pose at 5 Hz. Unlike
a live lidar feed, every frame comes from the same fixed PCD that the operator
approved for navigation.
"""

import argparse
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

MAP_FRAME = 'map'
POSE_TOPIC = '/nav/state_estimation'
OUTPUT_TOPIC = '/nav/static_cloud_map'


def load_pcd(path):
    """Read an ascii/binary XYZ PCD and return an (N, 3) float64 array."""
    if not os.path.isfile(path):
        raise RuntimeError('pcd file not found: %s' % path)

    header = {}
    with open(path, 'rb') as stream:
        while True:
            line = stream.readline().decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                break

        fields = header['FIELDS']
        sizes = [int(value) for value in header['SIZE']]
        types = header['TYPE']
        counts = [int(value) for value in header['COUNT']]
        num_points = int(header['POINTS'][0])
        storage = header['DATA'][0]

        if storage == 'ascii':
            table = np.loadtxt(stream, max_rows=num_points, ndmin=2)
            index = {name: i for i, name in enumerate(fields)}
            points = table[:, [index['x'], index['y'], index['z']]]
        elif storage == 'binary':
            kind = {'F': 'f', 'I': 'i', 'U': 'u'}
            dtype = np.dtype([
                (name, kind[kind_name] + str(size), (count,))
                for name, size, kind_name, count in zip(fields, sizes, types, counts)
            ])
            blob = stream.read(dtype.itemsize * num_points)
            if len(blob) < dtype.itemsize * num_points:
                raise RuntimeError('truncated pcd: %s' % path)
            table = np.frombuffer(blob, dtype=dtype)
            points = np.stack(
                [table['x'][:, 0], table['y'][:, 0], table['z'][:, 0]], axis=1)
        else:
            raise RuntimeError('unsupported pcd storage %r' % storage)

    return np.ascontiguousarray(points.reshape(-1, 3), dtype=np.float64)


def voxel_downsample(points, size):
    """Keep one deterministic point per cubic voxel, matching map_sim."""
    if size <= 0.0 or not len(points):
        return points
    keys = np.floor(points / size).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, keep = np.unique(flat, return_index=True)
    return points[keep]


def window_points(points, pose, window_size):
    """Return all points in a square centered on pose: window_size by window_size."""
    if not len(points):
        return points
    x, y, _ = pose
    half_size = float(window_size) * 0.5
    dx = points[:, 0] - x
    dy = points[:, 1] - y
    return points[(np.abs(dx) < half_size) & (np.abs(dy) < half_size)]


def cloud_msg(points, frame_id, stamp):
    """Build the same XYZI PointCloud2 layout used by map_sim."""
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name=name, offset=4 * index,
                   datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(('x', 'y', 'z', 'intensity'))
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 16 * len(points)
    data = np.zeros((len(points), 4), dtype=np.float32)
    data[:, :3] = points
    message.data = data.tobytes()
    message.is_dense = True
    return message


class StaticMapReplay(Node):

    def __init__(self, args):
        super().__init__('static_map_replay')
        started = time.monotonic()
        raw = load_pcd(args.map)
        self.points = voxel_downsample(raw, args.voxel)
        self.window_size = float(args.window_size)
        self.pose_timeout = float(args.pose_timeout)
        self.map_frame = args.frame
        self.pose = None
        self.pose_stamp = 0.0
        self.published = 0

        self.publisher = self.create_publisher(PointCloud2, args.output_topic, 5)
        self.create_subscription(Odometry, args.pose_topic, self.on_pose, 10)
        self.create_timer(1.0 / args.rate, self.publish_window)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            'static cloud: %d points after %.2f m voxel from %s; '
            'window %.1f x %.1f m at %.1f Hz in %.1f s' %
            (len(self.points), args.voxel, args.map, self.window_size,
             self.window_size,
             args.rate, time.monotonic() - started))
        self.get_logger().info(
            'waiting for %s before publishing %s' %
            (args.pose_topic, args.output_topic))

    def on_pose(self, message):
        position = message.pose.pose.position
        self.pose = (float(position.x), float(position.y), float(position.z))
        self.pose_stamp = time.monotonic()

    def publish_window(self):
        if self.pose is None:
            return
        if time.monotonic() - self.pose_stamp > self.pose_timeout:
            return
        points = window_points(self.points, self.pose, self.window_size)
        if not len(points):
            return
        self.publisher.publish(
            cloud_msg(points, self.map_frame, self.get_clock().now().to_msg()))
        self.published += 1

    def report(self):
        self.get_logger().info(
            'static cloud frames=%d pose=%s' %
            (self.published, 'none' if self.pose is None else
             '(%.2f, %.2f, %.2f)' % self.pose))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--pose-topic', default=POSE_TOPIC)
    parser.add_argument('--output-topic', default=OUTPUT_TOPIC)
    parser.add_argument('--frame', default=MAP_FRAME)
    parser.add_argument('--voxel', type=float, default=0.08)
    parser.add_argument('--window-size', type=float, default=6.0)
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument('--pose-timeout', type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = StaticMapReplay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
