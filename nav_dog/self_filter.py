#!/usr/bin/env python3
"""Transform Go2's registered cloud to map and remove body self-returns.

LIO-SAM publishes ``cloud_registered`` in its odometry frame, while the
obstacle stack consumes map-frame points.  The Mid-360 also sees the dog body
at about 0.35-0.70 m from the sensor, which blocks every localPlanner sweep.
This node performs both operations before terrainAnalysis:

    cloud_registered [odom] -> map TF -> blank self cylinder -> map cloud

If the TF is unavailable, the input cloud is not forwarded.  Forwarding an
untransformed cloud would silently reintroduce the coordinate mismatch.
"""
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


class CloudFrameSelfFilter(Node):
    def __init__(self):
        super().__init__('go2_cloud_frame_self_filter')
        self.target_frame = str(self.declare_parameter('target_frame', 'map').value)
        self.radius = float(self.declare_parameter('self_radius', 0.70).value)
        self.z_band = float(self.declare_parameter('self_z_band', 0.65).value)
        self.input_topic = str(self.declare_parameter(
            'input_topic', '/lio_sam_ros2/mapping/cloud_registered').value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/nav/cloud_registered_map').value)
        self.robot_position = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/lio_sam_ros2/mapping/re_location_odometry',
                                 self.odom_cb, 10)
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, qos)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.received = 0
        self.published = 0
        self.dropped_self = 0
        self.tf_failures = 0
        self.last_report = self.get_clock().now()
        self.get_logger().info(
            'cloud preprocess: %s -> %s, self_radius=%.2f m, z_band=%.2f m'
            % (self.input_topic, self.output_topic, self.radius, self.z_band))

    def odom_cb(self, message):
        position = message.pose.pose.position
        self.robot_position = np.array(
            [position.x, position.y, position.z], dtype=np.float64)

    @staticmethod
    def _rotation_matrix(quaternion):
        x, y, z, w = (float(quaternion.x), float(quaternion.y),
                      float(quaternion.z), float(quaternion.w))
        norm = x * x + y * y + z * z + w * w
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError('invalid transform quaternion')
        scale = 2.0 / norm
        xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
        xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
        wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
        return np.array([
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ], dtype=np.float64)

    @staticmethod
    def _structured_cloud(message):
        if message.point_step <= 0:
            raise ValueError('point_step must be positive')
        names, formats, offsets = [], [], []
        for field in message.fields:
            names.append(field.name)
            offsets.append(field.offset)
            formats.append('<f4')
        if not {'x', 'y', 'z'}.issubset(names):
            raise ValueError('cloud must contain x/y/z fields')
        dtype = np.dtype({'names': names, 'formats': formats,
                          'offsets': offsets, 'itemsize': message.point_step})
        points = np.frombuffer(bytearray(message.data), dtype=dtype).copy()
        expected = int(message.width) * max(1, int(message.height))
        return points[:min(points.size, expected)] if expected else points

    def _transform(self, source_frame):
        transform = self.tf_buffer.lookup_transform(
            self.target_frame, source_frame, Time())
        translation = transform.transform.translation
        rotation = self._rotation_matrix(transform.transform.rotation)
        return np.array([translation.x, translation.y, translation.z],
                        dtype=np.float64), rotation

    def cloud_cb(self, message):
        self.received += 1
        source_frame = message.header.frame_id or self.target_frame
        try:
            points = self._structured_cloud(message)
            if points.size == 0:
                self._publish(message, points)
                return
            xyz = np.column_stack((
                points['x'].astype(np.float64),
                points['y'].astype(np.float64),
                points['z'].astype(np.float64),
            ))
            if source_frame == self.target_frame:
                translation = np.zeros(3, dtype=np.float64)
                rotation = np.eye(3, dtype=np.float64)
            else:
                translation, rotation = self._transform(source_frame)
            transformed = xyz @ rotation.T + translation
            center = (self.robot_position if self.robot_position is not None
                      else translation)

            finite = np.all(np.isfinite(transformed), axis=1)
            dx = transformed[:, 0] - center[0]
            dy = transformed[:, 1] - center[1]
            dz = transformed[:, 2] - center[2]
            inside_self = (
                finite &
                (dx * dx + dy * dy <= self.radius * self.radius) &
                (np.abs(dz) <= self.z_band)
            )
            keep = finite & ~inside_self
            self.dropped_self += int(inside_self.sum())

            filtered = points[keep]
            if filtered.size:
                filtered['x'] = transformed[keep, 0].astype(filtered['x'].dtype)
                filtered['y'] = transformed[keep, 1].astype(filtered['y'].dtype)
                filtered['z'] = transformed[keep, 2].astype(filtered['z'].dtype)
            self._publish(message, filtered)
        except Exception as error:  # TF not ready is expected during startup.
            self.tf_failures += 1
            self.get_logger().warn(
                'drop cloud until map transform is ready: %s' % error,
                throttle_duration_sec=2.0)
        finally:
            self._report()

    def _publish(self, source, points):
        output = PointCloud2()
        output.header.stamp = source.header.stamp
        output.header.frame_id = self.target_frame
        output.height = 1
        output.width = int(points.size)
        output.fields = source.fields
        output.is_bigendian = source.is_bigendian
        output.point_step = source.point_step
        output.row_step = output.width * source.point_step
        output.data = points.tobytes()
        output.is_dense = True
        self.publisher.publish(output)
        self.published += 1

    def _report(self):
        now = self.get_clock().now()
        if (now - self.last_report).nanoseconds < 5_000_000_000:
            return
        self.last_report = now
        self.get_logger().info(
            'cloud preprocess: received=%d published=%d tf_failures=%d '
            'dropped_self_points=%d'
            % (self.received, self.published, self.tf_failures,
               self.dropped_self))


def main():
    rclpy.init()
    node = CloudFrameSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
