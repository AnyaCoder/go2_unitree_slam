#!/usr/bin/env python3
"""Publish a smooth map-frame navigation odometry from LIO-SAM outputs.

LIO-SAM's global localization and registered cloud are tied to its map-update
rate (about 3 Hz on the Go2). Its IMU preintegration output is much faster
(about 150 Hz). This bridge keeps the global localization as a low-rate
correction and propagates the high-rate motion between corrections:

    global = correction * imu_incremental

The output is rate-limited to 50 Hz for localPlanner/pathFollower. No state is
published until a global map correction exists. If either input goes stale,
the bridge asserts pathFollower's safety-stop bits instead of allowing the
controller to act on a stale pose.
"""
import collections
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8

from nav_pose_math import (
    Pose,
    blend_pose,
    compose,
    correction_from_global,
    normalize_quaternion,
    pose_distance,
    rotate_vector,
)


HighSample = collections.namedtuple('HighSample', 'stamp pose message received')


def message_pose(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    rotation = normalize_quaternion((
        orientation.x, orientation.y, orientation.z, orientation.w))
    return Pose(position.x, position.y, position.z, *rotation)


class NavigationPoseBridge(Node):
    def __init__(self):
        super().__init__('go2_nav_pose_bridge')
        self.high_topic = str(self.declare_parameter(
            'high_rate_topic', '/odometry/imu_incremental').value)
        self.global_topic = str(self.declare_parameter(
            'global_topic', '/lio_sam_ros2/mapping/re_location_odometry').value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/nav/state_estimation').value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 50.0).value)
        self.high_timeout = float(self.declare_parameter(
            'high_rate_timeout', 0.30).value)
        self.global_timeout = float(self.declare_parameter(
            'global_timeout', 1.00).value)
        self.correction_smoothing = float(self.declare_parameter(
            'correction_smoothing_sec', 0.15).value)
        self.sample_match_window = float(self.declare_parameter(
            'sample_match_window', 0.50).value)
        self.stop_on_stale = bool(self.declare_parameter(
            'stop_on_stale', True).value)
        if self.publish_hz <= 0.0:
            raise ValueError('publish_hz must be positive')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.high_samples = collections.deque(maxlen=64)
        self.last_high_received = 0.0
        self.last_global_received = 0.0
        self.last_global_stamp = None
        self.target_correction = None
        self.current_correction = None
        self.last_timer_time = time.monotonic()
        self.last_report_time = time.monotonic()
        self.high_received = 0
        self.global_received = 0
        self.published = 0
        self.stop_state = None

        self.create_subscription(Odometry, self.high_topic, self.high_rate_cb, qos)
        self.create_subscription(Odometry, self.global_topic, self.global_cb, qos)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.stop_publisher = self.create_publisher(Int8, '/stop', 10)
        self.create_timer(1.0 / self.publish_hz, self.publish_cb)
        self.get_logger().info(
            'nav pose bridge: %s + %s -> %s at %.0f Hz'
            % (self.high_topic, self.global_topic, self.output_topic,
               self.publish_hz))

    def _stamp(self, message):
        return (message.header.stamp.sec +
                message.header.stamp.nanosec * 1e-9)

    def high_rate_cb(self, message):
        try:
            pose = message_pose(message)
        except ValueError:
            return
        now = time.monotonic()
        sample = HighSample(self._stamp(message), pose, message, now)
        self.high_samples.append(sample)
        self.last_high_received = now
        self.high_received += 1

    def _nearest_high(self, stamp):
        if not self.high_samples:
            return None
        best = min(self.high_samples, key=lambda item: abs(item.stamp - stamp))
        if abs(best.stamp - stamp) <= self.sample_match_window:
            return best
        return self.high_samples[-1]

    def global_cb(self, message):
        stamp = self._stamp(message)
        high = self._nearest_high(stamp)
        if high is None:
            return
        try:
            global_pose = message_pose(message)
            correction = correction_from_global(global_pose, high.pose)
        except ValueError:
            return
        self.target_correction = correction
        self.last_global_received = time.monotonic()
        self.last_global_stamp = stamp
        self.global_received += 1
        if self.current_correction is None:
            self.current_correction = correction

    def _healthy(self, now):
        return (
            self.target_correction is not None
            and now - self.last_high_received <= self.high_timeout
            and now - self.last_global_received <= self.global_timeout
            and bool(self.high_samples)
        )

    def _set_stop(self, active):
        desired = 15 if active else 0
        if self.stop_state == desired:
            return
        self.stop_state = desired
        message = Int8()
        message.data = desired
        self.stop_publisher.publish(message)

    def _advance_correction(self, now):
        if self.current_correction is None:
            self.current_correction = self.target_correction
            return self.current_correction
        elapsed = max(0.0, now - self.last_timer_time)
        if self.correction_smoothing <= 0.0:
            alpha = 1.0
        else:
            alpha = min(1.0, elapsed / self.correction_smoothing)
        self.current_correction = blend_pose(
            self.current_correction, self.target_correction, alpha)
        return self.current_correction

    def publish_cb(self):
        now = time.monotonic()
        self.last_timer_time = now
        healthy = self._healthy(now)
        if self.stop_on_stale:
            self._set_stop(not healthy)
        if not healthy:
            self._report(now, healthy=False)
            return

        correction = self._advance_correction(now)
        sample = self.high_samples[-1]
        pose = compose(correction, sample.pose)
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = pose.z
        message.pose.pose.orientation.x = pose.qx
        message.pose.pose.orientation.y = pose.qy
        message.pose.pose.orientation.z = pose.qz
        message.pose.pose.orientation.w = pose.qw
        message.pose.covariance = list(sample.message.pose.covariance)

        linear = sample.message.twist.twist.linear
        angular = sample.message.twist.twist.angular
        rotated_linear = rotate_vector(correction[3:], (linear.x, linear.y, linear.z))
        rotated_angular = rotate_vector(correction[3:], (angular.x, angular.y, angular.z))
        message.twist.twist.linear.x = rotated_linear[0]
        message.twist.twist.linear.y = rotated_linear[1]
        message.twist.twist.linear.z = rotated_linear[2]
        message.twist.twist.angular.x = rotated_angular[0]
        message.twist.twist.angular.y = rotated_angular[1]
        message.twist.twist.angular.z = rotated_angular[2]
        message.twist.covariance = list(sample.message.twist.covariance)

        self.publisher.publish(message)
        self.published += 1
        self._report(now, healthy=True)

    def _report(self, now, healthy):
        if now - self.last_report_time < 5.0:
            return
        self.last_report_time = now
        correction_age = (now - self.last_global_received
                          if self.last_global_received else float('inf'))
        high_age = (now - self.last_high_received
                    if self.last_high_received else float('inf'))
        correction_error = (pose_distance(self.current_correction,
                                          self.target_correction)
                            if self.current_correction is not None else -1.0)
        self.get_logger().info(
            'nav pose bridge: healthy=%s high=%d global=%d published=%d '
            'high_age=%.3fs global_age=%.3fs correction_delta=%.3fm'
            % (healthy, self.high_received, self.global_received, self.published,
               high_age, correction_age, correction_error))


def main():
    rclpy.init()
    node = NavigationPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.stop_on_stale:
            node._set_stop(True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
