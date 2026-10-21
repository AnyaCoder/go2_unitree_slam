#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fail-closed execution gate between pathFollower and the Go2 Sport API.

pathFollower is remapped to /nav/path_follower_request.  Only a Move backed by
a current localization epoch, fresh IMU/odometry, a FAR Goalpoint and waypoint,
and a fresh path generated for that goal can reach /api/sport/request.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from unitree_api.msg import Request

from safety_state import (
    ROBOT_SPORT_API_ID_STOPMOVE,
    SafetyConfig,
    SafetyState,
    decode_sport_request,
)


def yaw_from_quaternion(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class NavSafetyGate(Node):

    def __init__(self, options):
        super().__init__('nav_safety_gate')
        self.options = options
        self.state = SafetyState(SafetyConfig(
            imu_calibration_window_s=options.imu_calibration_window,
            imu_stale_s=options.imu_stale,
            odom_stale_s=options.odom_stale,
            path_stale_s=options.path_stale,
            max_pose_to_initial_pose_m=options.max_pose_to_initial_pose,
        ))

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sport_pub = self.create_publisher(Request, options.sport_api_topic, 10)
        self.ready_pub = self.create_publisher(Bool, options.ready_topic, latched)
        self.status_pub = self.create_publisher(String, options.status_topic, latched)

        self.create_subscription(Imu, options.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        self.create_subscription(Odometry, options.odom_topic, self.on_odom, 10)
        self.create_subscription(Log, '/rosout', self.on_rosout, 50)
        self.create_subscription(PointStamped, '/goal_point', self.on_goal, 10)
        self.create_subscription(PointStamped, '/way_point', self.on_waypoint, 10)
        self.create_subscription(Path, '/path', self.on_path, 10)
        self.create_subscription(Request, options.path_follower_topic, self.on_path_follower_request, 10)

        self.motion_ready_previous = False
        self.last_status_reason = ''
        self.last_ready_publish = 0.0
        self.last_status_publish = 0.0
        self.last_stop_publish = 0.0
        self.last_move_forward = None
        self.last_path_follower_request = 0.0
        self.blocked_move_seen = 0.0
        self.blocked_move_count = 0
        self.forwarded_move_count = 0
        self.forwarded_stop_count = 0
        self.rejected_api_count = 0
        self.create_timer(0.10, self.tick)
        self.publish_readiness(force=True)
        self.get_logger().info(
            'SAFETY GATE: %s -> %s; fail-closed until IMU calibration, '
            'initial pose, localization, Goalpoint/waypoint and fresh path all pass'
            % (options.path_follower_topic, options.sport_api_topic))

    def on_imu(self, message):
        now = time.monotonic()
        gyro = (message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z)
        accel = (message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z)
        was_ready = self.state.imu_calibration.ready
        self.state.accept_imu(gyro, accel, now)
        if not was_ready and self.state.imu_calibration.ready:
            self.get_logger().info(self.state.imu_calibration.detail)

    def on_initial_pose(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (position.x, position.y, orientation.x, orientation.y,
                  orientation.z, orientation.w)
        now = time.monotonic()
        frame = message.header.frame_id
        quaternion_norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2 +
            orientation.z ** 2 + orientation.w ** 2)
        if (frame not in ('', 'map') or
                not all(math.isfinite(v) for v in values) or
                not 0.5 <= quaternion_norm <= 1.5):
            self.state.invalidate_initial_pose()
            self.get_logger().error(
                'INVALID INITIAL POSE (%s): all motion authorization cleared' % frame)
            return
        yaw = yaw_from_quaternion(orientation)
        if self.state.accept_initial_pose(position.x, position.y, yaw, now):
            self.get_logger().warn(
                'NEW LOCALIZATION EPOCH: initial pose x=%.3f y=%.3f yaw=%.3f rad; '
                'all old goals, waypoints and paths are invalidated' %
                (position.x, position.y, yaw))
        else:
            self.state.invalidate_initial_pose()
            self.get_logger().error('rejected /initialpose; motion authorization cleared')

    def on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (position.x, position.y, position.z, orientation.x,
                  orientation.y, orientation.z, orientation.w)
        quaternion_norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2 +
            orientation.z ** 2 + orientation.w ** 2)
        if not all(math.isfinite(v) for v in values) or not 0.5 <= quaternion_norm <= 1.5:
            self.state.invalidate_odom()
            self.get_logger().error(
                'invalid localization odometry; motion authorization cleared',
                throttle_duration_sec=2.0)
            return
        self.state.accept_odom(
            position.x, position.y, yaw_from_quaternion(orientation), time.monotonic())

    def on_rosout(self, message):
        text = message.msg or ''
        if 'Pose initialization successful' in text or 'Initializing Succeed' in text:
            now = time.monotonic()
            if self.state.accept_localization_success(now, text):
                self.get_logger().info('LOCALIZATION CONFIRMED after current initial pose')

    def on_goal(self, message):
        point = message.point
        now = time.monotonic()
        if message.header.frame_id not in ('', 'map'):
            self.state.invalidate_goal()
            self.get_logger().error(
                'Goalpoint frame %s is not map; cleared current goal' % message.header.frame_id)
            return
        if self.state.accept_goal(point.x, point.y, point.z, now):
            self.get_logger().info(
                'NEW GOAL accepted in current localization epoch: x=%.3f y=%.3f z=%.3f'
                % (point.x, point.y, point.z))
        else:
            self.state.invalidate_goal()
            self.get_logger().warn(
                'Goalpoint rejected until IMU/initial-pose/localization are ready; '
                'existing goal cleared', throttle_duration_sec=2.0)

    def on_waypoint(self, message):
        point = message.point
        if message.header.frame_id not in ('', 'map'):
            self.state.invalidate_waypoint()
            self.get_logger().error(
                'waypoint frame %s is not map; cleared current waypoint' %
                message.header.frame_id)
            return
        if not self.state.accept_waypoint(point.x, point.y, point.z, time.monotonic()):
            self.state.invalidate_waypoint()
            self.get_logger().warn(
                'FAR waypoint rejected outside a ready localization epoch; '
                'current waypoint cleared', throttle_duration_sec=2.0)

    def on_path(self, message):
        if message.header.frame_id != 'vehicle':
            self.state.invalidate_path()
            self.get_logger().error(
                'local path frame %r is not vehicle; path authorization cleared' %
                message.header.frame_id, throttle_duration_sec=2.0)
            return
        self.state.accept_path(
            [pose.pose.position for pose in message.poses], time.monotonic())

    def on_path_follower_request(self, message):
        now = time.monotonic()
        self.last_path_follower_request = now
        api_id = int(message.header.identity.api_id)
        try:
            kind, values = decode_sport_request(api_id, message.parameter)
        except ValueError as error:
            self.rejected_api_count += 1
            self.get_logger().error('BLOCKED: %s' % error, throttle_duration_sec=2.0)
            return

        if kind == 'stop':
            # StopMove is safe and must pass so pathFollower can stop at an
            # obstacle or goal immediately.
            self.publish_stop()
            return

        motion_ready, reasons = self.state.evaluate(now)
        if not motion_ready:
            self.blocked_move_seen = now
            self.blocked_move_count += 1
            self.get_logger().error(
                'BLOCKED pathFollower Move: %s' % '; '.join(reasons),
                throttle_duration_sec=2.0)
            self.publish_stop()
            return

        self.sport_pub.publish(message)
        self.last_move_forward = now
        self.forwarded_move_count += 1

    def publish_stop(self):
        now = time.monotonic()
        if now - self.last_stop_publish < 0.10:
            return
        request = Request()
        request.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE
        self.sport_pub.publish(request)
        self.last_stop_publish = now
        self.forwarded_stop_count += 1

    def publish_readiness(self, force=False):
        now = time.monotonic()
        ready = self.state.goal_commands_ready(now)
        if force or now - self.last_ready_publish >= 0.5:
            message = Bool()
            message.data = ready
            self.ready_pub.publish(message)
            self.last_ready_publish = now

        status = self.state.status(now)
        status['gate'] = {
            'path_follower_topic': self.options.path_follower_topic,
            'sport_api_topic': self.options.sport_api_topic,
            'forwarded_move': self.forwarded_move_count,
            'forwarded_stop': self.forwarded_stop_count,
            'blocked_move': self.blocked_move_count,
            'rejected_api': self.rejected_api_count,
        }
        if force or now - self.last_status_publish >= 1.0:
            message = String()
            message.data = json.dumps(status, sort_keys=True, separators=(',', ':'))
            self.status_pub.publish(message)
            self.last_status_publish = now

        reason = (status['reasons'][0] if status['reasons'] else 'READY_FOR_MOTION')
        reason_key = reason.split(':', 1)[0]
        if force or reason_key != self.last_status_reason:
            if status['motion_ready']:
                self.get_logger().info('SAFETY READY: Goalpoint motion is authorized')
            else:
                self.get_logger().warn('SAFETY DISARMED: %s' % reason)
            self.last_status_reason = reason_key

    def tick(self):
        now = time.monotonic()
        motion_ready, reasons = self.state.evaluate(now)
        if self.motion_ready_previous and not motion_ready:
            self.get_logger().error(
                'SAFETY REVOKED while moving: %s' % '; '.join(reasons))
            self.publish_stop()
        self.motion_ready_previous = motion_ready

        # If pathFollower is trying to move while disarmed, keep replacing its
        # request with StopMove.  We do not spam StopMove when pathFollower is
        # idle, so the physical remote controller remains independent.
        if not motion_ready and now - self.blocked_move_seen <= 0.6:
            self.publish_stop()

        # Once a Move was allowed, absence of pathFollower commands is itself a
        # stale-control condition.  Send one StopMove after the stream ends.
        if self.last_move_forward is not None and now - self.last_move_forward > 0.5:
            if now - self.last_path_follower_request > 0.5:
                self.publish_stop()
                self.last_move_forward = None

        self.publish_readiness()

    def shutdown_stop(self):
        # A second Ctrl+C may arrive while the launch service is stopping.
        # Still try to emit StopMove, but never mask the original exit reason.
        for _ in range(3):
            try:
                self.publish_stop()
                time.sleep(0.03)
            except BaseException:
                break


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path-follower-topic', default='/nav/path_follower_request')
    parser.add_argument('--sport-api-topic', default='/api/sport/request')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--odom-topic', default='/lio_sam_ros2/mapping/re_location_odometry')
    parser.add_argument('--ready-topic', default='/nav/goal_commands_ready')
    parser.add_argument('--status-topic', default='/nav/safety_status')
    parser.add_argument('--imu-calibration-window', type=float, default=2.5)
    parser.add_argument('--imu-stale', type=float, default=1.0)
    parser.add_argument('--odom-stale', type=float, default=1.0)
    parser.add_argument('--path-stale', type=float, default=3.0)
    parser.add_argument('--max-pose-to-initial-pose', type=float, default=5.0)
    options, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = NavSafetyGate(options)
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().fatal('safety gate crashed: %s' % error)
        exit_code = 1
    finally:
        try:
            node.shutdown_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
