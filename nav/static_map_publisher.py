#!/usr/bin/env python3
"""Publish the fixed PCD map as ``/overall_map`` for local RViz viewing.

The final architecture has no live sensor-cloud uplink. Both simulation and
real-robot RViz use this PCD background so the displayed scene is identical.
"""

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from far_viz_receiver import make_cloud, read_pcd


class StaticMapPublisher(Node):

    def __init__(self, map_path, frame_id='map', publish_period=1.0):
        super().__init__('static_map_publisher')
        xyz, intensity, rgb = read_pcd(map_path)
        self.cloud = make_cloud(
            xyz, intensity, frame_id, self.get_clock().now().to_msg(), rgb=rgb)
        self.publisher = self.create_publisher(PointCloud2, '/overall_map', 2)
        self.create_timer(publish_period, self.publish_map)
        self.get_logger().info(
            'static map: %d points from %s' % (len(xyz), map_path))

    def publish_map(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.cloud)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--period', type=float, default=1.0)
    options, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = StaticMapPublisher(options.map, options.frame, options.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
