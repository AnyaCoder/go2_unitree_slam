#!/usr/bin/env python3
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nav_dog'))

from safety_state import (  # noqa: E402
    ROBOT_SPORT_API_ID_MOVE,
    ROBOT_SPORT_API_ID_STOPMOVE,
    SafetyConfig,
    SafetyState,
    decode_sport_request,
)


class SafetyStateTest(unittest.TestCase):

    def setUp(self):
        self.cfg = SafetyConfig(
            imu_calibration_window_s=0.5,
            imu_min_samples=10,
            imu_max_sample_gap_s=0.1,
            imu_stale_s=0.5,
            odom_stale_s=0.5,
            path_stale_s=0.3,
            max_pose_to_initial_pose_m=0.75,
        )
        self.state = SafetyState(self.cfg)

    def calibrate(self, end=0.55):
        for i in range(int(end / 0.05) + 1):
            self.state.accept_imu((0.001, -0.002, 0.003), (0.01, 0.02, 9.78), i * 0.05)
        self.assertTrue(self.state.imu_calibration.ready, self.state.imu_calibration)

    def ready_sequence(self, now=2.0):
        self.calibrate(0.55)
        self.assertTrue(self.state.accept_initial_pose(1.0, 2.0, 0.0, 1.0))
        self.assertTrue(self.state.accept_localization_success(1.2, 'Pose initialization successful'))
        self.assertTrue(self.state.accept_odom(1.05, 2.02, 0.0, now))
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), now)
        self.assertTrue(self.state.accept_goal(3.0, 4.0, 0.0, now))
        self.assertTrue(self.state.accept_waypoint(2.5, 3.0, 0.0, now))
        points = [SimpleNamespace(x=1.0, y=2.0, z=0.0), SimpleNamespace(x=2.0, y=3.0, z=0.0)]
        self.assertTrue(self.state.accept_path(points, now))

    def test_fresh_gate_is_disarmed(self):
        ready, reasons = self.state.evaluate(0.0)
        self.assertFalse(ready)
        self.assertIn('IMU_NOT_CALIBRATED', reasons[0])
        self.assertFalse(self.state.goal_commands_ready(0.0))

    def test_static_imu_calibration_and_goal_sequence_arm_motion(self):
        self.ready_sequence()
        ready, reasons = self.state.evaluate(2.05)
        self.assertTrue(ready, reasons)
        self.assertTrue(self.state.goal_commands_ready(2.05))

    def test_goal_before_initial_pose_is_rejected(self):
        self.calibrate()
        self.assertFalse(self.state.accept_goal(1.0, 2.0, 0.0, 1.0))
        self.assertIsNone(self.state.goal_xyz)

    def test_new_initial_pose_invalidates_old_goal_and_path(self):
        self.ready_sequence()
        self.assertTrue(self.state.evaluate(2.05)[0])
        self.assertTrue(self.state.accept_initial_pose(0.5, 0.5, 0.0, 2.1))
        ready, reasons = self.state.evaluate(2.15)
        self.assertFalse(ready)
        self.assertIn('LOCALIZATION_NOT_CONFIRMED', reasons[0])
        self.assertIsNone(self.state.goal_xyz)
        self.assertIsNone(self.state.path_time)

    def test_stale_path_revokes_motion(self):
        self.ready_sequence()
        ready, _ = self.state.evaluate(2.05)
        self.assertTrue(ready)
        ready, reasons = self.state.evaluate(2.50)
        self.assertFalse(ready)
        self.assertIn('PATH_STALE', reasons[0])

    def test_position_anchor_is_not_rechecked_after_motion(self):
        self.ready_sequence()
        self.assertTrue(self.state.evaluate(2.05)[0])
        # The selected initial pose is at (1, 2).  A legitimate long run may
        # move far beyond the one-time 0.75 m acquisition tolerance.
        self.assertTrue(self.state.accept_odom(20.0, 20.0, 0.0, 2.2))
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 2.2)
        points = [SimpleNamespace(x=1.0, y=2.0, z=0.0)]
        self.assertTrue(self.state.accept_path(points, 2.2))
        ready, reasons = self.state.evaluate(2.25)
        self.assertTrue(ready, reasons)

    def test_goal_and_waypoint_do_not_expire(self):
        self.ready_sequence()
        self.assertTrue(self.state.evaluate(2.05)[0])
        # Neither a still-active FAR goal nor an unchanged FAR waypoint may be
        # invalidated merely because the dog has been running for a long time.
        later = 301.2
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), later)
        self.assertTrue(self.state.accept_odom(2.0, 3.0, 0.0, later))
        points = [SimpleNamespace(x=1.0, y=2.0, z=0.0)]
        self.assertTrue(self.state.accept_path(points, later))
        ready, reasons = self.state.evaluate(later + 0.05)
        self.assertTrue(ready, reasons)

    def test_new_goal_requires_new_waypoint_and_path(self):
        self.ready_sequence()
        self.assertTrue(self.state.evaluate(2.05)[0])
        self.assertTrue(self.state.accept_goal(4.0, 5.0, 0.0, 2.1))
        ready, reasons = self.state.evaluate(2.15)
        self.assertFalse(ready)
        self.assertIn('WAYPOINT_REQUIRED', reasons[0])
        points = [SimpleNamespace(x=1.0, y=2.0, z=0.0)]
        self.assertTrue(self.state.accept_path(points, 2.15))
        ready, reasons = self.state.evaluate(2.16)
        self.assertFalse(ready)
        self.assertIn('WAYPOINT_REQUIRED', reasons[0])
        self.assertTrue(self.state.accept_waypoint(3.0, 4.0, 0.0, 2.15))
        ready, reasons = self.state.evaluate(2.16)
        self.assertTrue(ready, reasons)

    def test_new_goal_path_cannot_reuse_previous_path(self):
        self.ready_sequence()
        self.assertTrue(self.state.evaluate(2.05)[0])
        self.assertTrue(self.state.accept_goal(4.0, 5.0, 0.0, 2.1))
        self.assertTrue(self.state.accept_waypoint(3.0, 4.0, 0.0, 2.1))
        ready, reasons = self.state.evaluate(2.15)
        self.assertFalse(ready)
        self.assertIn('PATH_REQUIRED', reasons[0])

    def test_missing_goal_blocks_new_path(self):
        self.calibrate()
        self.state.accept_initial_pose(1.0, 2.0, 0.0, 1.0)
        self.state.accept_localization_success(1.2)
        self.state.accept_odom(1.0, 2.0, 0.0, 2.0)
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 2.0)
        points = [SimpleNamespace(x=1.0, y=2.0, z=0.0)]
        self.assertFalse(self.state.accept_path(points, 2.0))
        ready, reasons = self.state.evaluate(2.0)
        self.assertFalse(ready)
        self.assertIn('GOAL_REQUIRED', reasons[0])

    def test_gyro_bias_fails_calibration(self):
        for i in range(30):
            self.state.accept_imu((0.08, 0.0, 0.0), (0.0, 0.0, 9.8), i * 0.05)
        self.assertFalse(self.state.imu_calibration.ready)
        self.assertEqual(self.state.imu_calibration.state, 'GYRO_BIAS')

    def test_wrong_accel_scale_fails_calibration(self):
        for i in range(30):
            self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 0.995), i * 0.05)
        self.assertFalse(self.state.imu_calibration.ready)
        self.assertEqual(self.state.imu_calibration.state, 'ACCEL_SCALE')

    def test_pose_far_from_selected_initial_pose_is_blocked(self):
        self.calibrate()
        self.state.accept_initial_pose(1.0, 2.0, 0.0, 1.0)
        self.state.accept_localization_success(1.2)
        self.state.accept_odom(3.0, 4.0, 0.0, 2.0)
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 2.0)
        ready, reasons = self.state.evaluate(2.0)
        self.assertFalse(ready)
        self.assertIn('POSE_NOT_ANCHORED', reasons[0])

    def test_yaw_far_from_selected_initial_pose_is_blocked(self):
        self.calibrate()
        self.state.accept_initial_pose(1.0, 2.0, 0.0, 1.0)
        self.state.accept_localization_success(1.2)
        self.state.accept_odom(1.0, 2.0, 2.0, 2.0)
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 2.0)
        ready, reasons = self.state.evaluate(2.0)
        self.assertFalse(ready)
        self.assertIn('ORIENTATION_NOT_ANCHORED', reasons[0])

    def test_localization_anchor_waits_for_successful_relocation(self):
        self.calibrate()
        self.state.accept_initial_pose(1.0, 2.0, 0.0, 1.0)
        # An odom sample arriving before relocation success must not anchor the
        # new epoch, even if it happens to be near the clicked initial pose.
        self.state.accept_odom(1.0, 2.0, 0.0, 1.1)
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 1.1)
        ready, reasons = self.state.evaluate(1.1)
        self.assertFalse(ready)
        self.assertIn('LOCALIZATION_NOT_CONFIRMED', reasons[0])
        self.assertFalse(self.state.localization_anchor_valid)

        self.state.accept_localization_success(1.2)
        self.state.accept_odom(20.0, 20.0, 0.0, 1.3)
        self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 1.3)
        ready, reasons = self.state.evaluate(1.3)
        self.assertFalse(ready)
        self.assertIn('POSE_NOT_ANCHORED', reasons[0])
        self.assertFalse(self.state.localization_anchor_valid)

    def test_static_path_refresh_does_not_periodically_disarm(self):
        # Match the deployed threshold: localPlanner refreshes path at about
        # 1 Hz, while safety allows up to 3 s of path age.
        self.state = SafetyState(replace(self.cfg, path_stale_s=3.0))
        self.ready_sequence()
        points = [SimpleNamespace(x=1.0, y=2.0, z=0.0)]
        for cycle in range(10):
            now = 2.0 + cycle
            self.assertTrue(self.state.accept_path(points, now))
            self.state.accept_imu((0.0, 0.0, 0.0), (0.0, 0.0, 9.8), now + 0.8)
            self.state.accept_odom(1.1, 2.0, 0.0, now + 0.8)
            ready, reasons = self.state.evaluate(now + 0.9)
            self.assertTrue(ready, (cycle, reasons))

    def test_invalid_or_unexpected_sport_api_is_rejected(self):
        self.assertEqual(decode_sport_request(ROBOT_SPORT_API_ID_STOPMOVE, ''), ('stop', None))
        kind, values = decode_sport_request(ROBOT_SPORT_API_ID_MOVE, '{"x":0.2,"y":0,"z":0.1}')
        self.assertEqual(kind, 'move')
        self.assertAlmostEqual(values[0], 0.2)
        with self.assertRaises(ValueError):
            decode_sport_request(1004, '{}')
        with self.assertRaises(ValueError):
            decode_sport_request(ROBOT_SPORT_API_ID_MOVE, '{"x":NaN,"y":0,"z":0}')


if __name__ == '__main__':
    unittest.main()
