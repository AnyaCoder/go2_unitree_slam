#!/usr/bin/env python3
"""Repeat LIO-SAM's map-frame relocalization pose for the navigation stack.

On this Go2 the IMU-preintegration odometry keeps moving while the robot is
stationary. It must not be used to propagate the global localization pose.
The relocalization odometry is the authoritative pose, so this node repeats the
newest accepted map pose at the planner's required output rate.

No state is published until relocalization exists. If the input goes stale, the
node asserts pathFollower's safety-stop bits instead of allowing the controller
to act on a stale pose.
"""
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8


class NavigationPoseBridge(Node):
    def __init__(self):
        super().__init__('go2_nav_pose_bridge')
        self.input_topic = str(self.declare_parameter(
            'input_topic',
            '/lio_sam_ros2/mapping/re_location_odometry').value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/nav/state_estimation').value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 50.0).value)
        self.input_timeout = float(self.declare_parameter(
            'input_timeout', 1.00).value)
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
        self.latest_message = None
        self.last_input_received = 0.0
        self.last_input_stamp = None
        self.last_report_time = time.monotonic()
        self.input_received = 0
        self.published = 0
        self.stop_state = None

        self.create_subscription(Odometry, self.input_topic, self.input_cb, qos)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.stop_publisher = self.create_publisher(Int8, '/stop', 10)
        self.create_timer(1.0 / self.publish_hz, self.publish_cb)
        self.get_logger().info(
            'nav pose relay: %s -> %s at %.0f Hz (IMU odom disabled)'
            % (self.input_topic, self.output_topic, self.publish_hz))

    def _stamp(self, message):
        return (message.header.stamp.sec +
                message.header.stamp.nanosec * 1e-9)

    def input_cb(self, message):
        now = time.monotonic()
        self.latest_message = message
        self.last_input_received = now
        self.last_input_stamp = self._stamp(message)
        self.input_received += 1

    def _healthy(self, now):
        return (
            self.latest_message is not None
            and now - self.last_input_received <= self.input_timeout
        )

    def _set_stop(self, active):
        desired = 15 if active else 0
        if self.stop_state == desired:
            return
        self.stop_state = desired
        message = Int8()
        message.data = desired
        self.stop_publisher.publish(message)

    def publish_cb(self):
        now = time.monotonic()
        healthy = self._healthy(now)
        if self.stop_on_stale:
            self._set_stop(not healthy)
        if not healthy:
            self._report(now, healthy=False)
            return

        message = Odometry()
        message.pose = self.latest_message.pose
        message.twist = self.latest_message.twist
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.child_frame_id = 'base_link'

        self.publisher.publish(message)
        self.published += 1
        self._report(now, healthy=True)

    def _report(self, now, healthy):
        if now - self.last_report_time < 5.0:
            return
        self.last_report_time = now
        input_age = (now - self.last_input_received
                     if self.last_input_received else float('inf'))
        self.get_logger().info(
            'nav pose relay: healthy=%s input=%d published=%d '
            'input_age=%.3fs input_stamp=%s'
            % (healthy, self.input_received, self.published, input_age,
               self.last_input_stamp))


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
