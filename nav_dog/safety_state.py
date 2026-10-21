#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure safety state machine for Goalpoint-driven Go2 motion.

This module deliberately has no ROS dependency.  The runtime gate supplies
timestamped events and the state machine decides whether a pathFollower Move
request may reach the robot.  Keeping the policy separate makes the dangerous
cases (old path, missing initial pose, stale odometry, failed IMU calibration)
testable without publishing to a real robot.
"""

from collections import deque
from dataclasses import dataclass
import json
import math
from typing import Deque, Dict, List, Optional, Tuple


ROBOT_SPORT_API_ID_STOPMOVE = 1003
ROBOT_SPORT_API_ID_MOVE = 1008


@dataclass(frozen=True)
class SafetyConfig:
    # A fresh run must observe a stationary IMU window before Goalpoint works.
    imu_calibration_window_s: float = 2.5
    imu_min_samples: int = 100
    imu_max_sample_gap_s: float = 0.20
    imu_max_gyro_bias: float = 0.030       # rad/s, worst-axis mean
    imu_max_gyro_std: float = 0.020        # rad/s, worst-axis std
    imu_min_accel_norm: float = 9.00       # m/s^2
    imu_max_accel_norm: float = 10.60      # m/s^2
    imu_stale_s: float = 1.0

    initial_session_timeout_s: float = 180.0
    max_pose_to_initial_pose_m: float = 5.0
    max_yaw_to_initial_pose_rad: float = 1.0
    odom_stale_s: float = 1.0

    # localPlanner publishes /path only when a new terrain cloud arrives.
    # Real LIO terrain is about 1 Hz, so a 1 s threshold creates artificial
    # stop/start cycles.  Three seconds still catches a dead planner promptly.
    path_stale_s: float = 3.0


@dataclass
class ImuCalibration:
    ready: bool = False
    state: str = 'WAITING'
    samples: int = 0
    span_s: float = 0.0
    gyro_bias: Optional[float] = None
    gyro_std: Optional[float] = None
    accel_norm: Optional[float] = None
    detail: str = 'waiting for /livox/imu samples'


class SafetyState:
    """Latching readiness plus motion authorization for the current goal."""

    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self._imu_window: Deque[Tuple[float, float, float, float, float, float, float]] = deque()
        self.imu_calibration = ImuCalibration()
        self.imu_last_time = 0.0

        self.initial_pose_time: Optional[float] = None
        self.initial_pose_xy: Optional[Tuple[float, float]] = None
        self.initial_pose_yaw: Optional[float] = None
        self.localization_success_time: Optional[float] = None
        self.localization_anchor_valid = False
        self.odom_time = 0.0
        self.odom_xy: Optional[Tuple[float, float]] = None
        self.odom_yaw: Optional[float] = None

        self.goal_time: Optional[float] = None
        self.goal_xyz: Optional[Tuple[float, float, float]] = None
        self.waypoint_time: Optional[float] = None
        self.waypoint_xyz: Optional[Tuple[float, float, float]] = None
        self.path_time: Optional[float] = None
        self.path_points = 0

        self.last_rejection = ''
        self.last_order_rejection = ''

    @staticmethod
    def _finite3(values) -> bool:
        try:
            return len(values) == 3 and all(math.isfinite(float(v)) for v in values)
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _finite2(values) -> bool:
        try:
            return len(values) == 2 and all(math.isfinite(float(v)) for v in values)
        except (TypeError, ValueError):
            return False

    def accept_imu(self, gyro, accel, when: float) -> bool:
        """Consume one IMU sample and, when still, latch calibration health.

        The driver is responsible for applying its configured bias.  This gate
        verifies the resulting stream is actually plausible and stationary; a
        raw or mis-scaled IMU therefore cannot silently arm motion.
        """
        values = tuple(float(v) for v in (list(gyro) + list(accel))) if gyro is not None and accel is not None else ()
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            return False
        self.imu_last_time = when
        if self.imu_calibration.ready:
            return True

        self._imu_window.append((when,) + values)
        oldest = when - self.config.imu_calibration_window_s
        while self._imu_window and self._imu_window[0][0] < oldest:
            self._imu_window.popleft()
        self._evaluate_imu_calibration(when)
        return self.imu_calibration.ready

    def _evaluate_imu_calibration(self, now: float) -> None:
        cfg = self.config
        samples = list(self._imu_window)
        if not samples:
            self.imu_calibration = ImuCalibration()
            return

        span = samples[-1][0] - samples[0][0]
        gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        counts = len(samples)
        values = [row[1:] for row in samples]
        means = [sum(row[i] for row in values) / counts for i in range(6)]
        gyro_means = means[:3]
        accel_means = means[3:]
        gyro_std = max(
            math.sqrt(sum((row[i] - gyro_means[i]) ** 2 for row in values) / counts)
            for i in range(3)
        )
        gyro_bias = max(abs(v) for v in gyro_means)
        accel_norm = math.sqrt(sum(v * v for v in accel_means))

        self.imu_calibration = ImuCalibration(
            samples=counts,
            span_s=span,
            gyro_bias=gyro_bias,
            gyro_std=gyro_std,
            accel_norm=accel_norm,
        )
        if counts < cfg.imu_min_samples or span < cfg.imu_calibration_window_s * 0.90:
            self.imu_calibration.state = 'COLLECTING'
            self.imu_calibration.detail = (
                'collecting stationary IMU: samples=%d span=%.2fs' % (counts, span)
            )
            return
        if gaps and max(gaps) > cfg.imu_max_sample_gap_s:
            self.imu_calibration.state = 'STREAM_GAP'
            self.imu_calibration.detail = (
                'IMU stream has a %.3fs gap; keep the dog stationary' % max(gaps)
            )
            return
        if gyro_bias > cfg.imu_max_gyro_bias:
            self.imu_calibration.state = 'GYRO_BIAS'
            self.imu_calibration.detail = (
                'gyro bias %.4f rad/s exceeds %.4f; apply/fix Livox gyro calibration'
                % (gyro_bias, cfg.imu_max_gyro_bias)
            )
            return
        if gyro_std > cfg.imu_max_gyro_std:
            self.imu_calibration.state = 'NOT_STATIONARY'
            self.imu_calibration.detail = (
                'gyro std %.4f rad/s exceeds %.4f; keep the dog completely still'
                % (gyro_std, cfg.imu_max_gyro_std)
            )
            return
        if not (cfg.imu_min_accel_norm <= accel_norm <= cfg.imu_max_accel_norm):
            self.imu_calibration.state = 'ACCEL_SCALE'
            self.imu_calibration.detail = (
                'accel norm %.3f m/s^2 is outside [%.2f, %.2f]; check IMU units/calibration'
                % (accel_norm, cfg.imu_min_accel_norm, cfg.imu_max_accel_norm)
            )
            return

        self.imu_calibration.ready = True
        self.imu_calibration.state = 'READY'
        self.imu_calibration.detail = (
            'stationary IMU calibrated: samples=%d bias=%.4f rad/s std=%.4f '
            'accel=%.3f m/s^2' % (counts, gyro_bias, gyro_std, accel_norm)
        )

    def accept_initial_pose(self, x: float, y: float, yaw: float, when: float) -> bool:
        if not self._finite3((x, y, yaw)):
            return False
        self.initial_pose_time = when
        self.initial_pose_xy = (float(x), float(y))
        self.initial_pose_yaw = float(yaw)
        self.localization_success_time = None
        self.localization_anchor_valid = False
        # New relocation invalidates every old command and any old path.
        self._reset_motion_epoch()
        return True

    def invalidate_initial_pose(self) -> None:
        self.initial_pose_time = None
        self.initial_pose_xy = None
        self.initial_pose_yaw = None
        self.localization_success_time = None
        self.localization_anchor_valid = False
        self._reset_motion_epoch()

    def invalidate_odom(self) -> None:
        self.odom_time = 0.0
        self.odom_xy = None
        self.odom_yaw = None

    def invalidate_goal(self) -> None:
        self.goal_time = None
        self.goal_xyz = None
        self.path_time = None
        self.path_points = 0

    def invalidate_waypoint(self) -> None:
        self.waypoint_time = None
        self.waypoint_xyz = None

    def invalidate_path(self) -> None:
        self.path_time = None
        self.path_points = 0

    def accept_localization_success(self, when: float, message: str = '') -> bool:
        if self.initial_pose_time is None:
            return False
        if when < self.initial_pose_time:
            return False
        if when - self.initial_pose_time > self.config.initial_session_timeout_s:
            return False
        self.localization_success_time = when
        return True

    def accept_odom(self, x: float, y: float, yaw: float, when: float) -> bool:
        if not self._finite3((x, y, yaw)):
            return False
        self.odom_xy = (float(x), float(y))
        self.odom_yaw = float(yaw)
        self.odom_time = when
        return True

    def accept_goal(self, x: float, y: float, z: float, when: float) -> bool:
        if not self._finite3((x, y, z)):
            return False
        if not self.goal_commands_ready(when):
            self.last_order_rejection = 'goal received before IMU/pose/localization ready'
            return False
        self.goal_time = when
        self.goal_xyz = (float(x), float(y), float(z))
        # A new final goal must get a newly generated path and waypoint.  An
        # old path being republished by localPlanner cannot authorize motion.
        self.waypoint_time = None
        self.waypoint_xyz = None
        self.path_time = None
        self.path_points = 0
        return True

    def accept_waypoint(self, x: float, y: float, z: float, when: float) -> bool:
        if not self._finite3((x, y, z)):
            return False
        if not self.goal_commands_ready(when):
            self.last_order_rejection = 'waypoint received before IMU/pose/localization ready'
            return False
        self.waypoint_time = when
        self.waypoint_xyz = (float(x), float(y), float(z))
        return True

    def accept_path(self, points, when: float) -> bool:
        try:
            count = len(points)
        except TypeError:
            return False
        if count <= 0:
            self.path_time = None
            self.path_points = 0
            return False
        if self.goal_time is None:
            return False
        for point in points:
            try:
                values = (point.x, point.y, point.z)
            except AttributeError:
                try:
                    values = (point[0], point[1], point[2])
                except (TypeError, IndexError):
                    return False
            if not self._finite3(values):
                return False
        self.path_time = when
        self.path_points = count
        return True

    def _reset_motion_epoch(self) -> None:
        self.odom_time = 0.0
        self.odom_xy = None
        self.odom_yaw = None
        self.goal_time = None
        self.goal_xyz = None
        self.waypoint_time = None
        self.waypoint_xyz = None
        self.path_time = None
        self.path_points = 0

    def _localization_reasons(self, now: float) -> List[str]:
        cfg = self.config
        reasons: List[str] = []
        localization_confirmed = (
            self.initial_pose_time is not None and
            self.localization_success_time is not None and
            self.localization_success_time >= self.initial_pose_time
        )
        if not self.imu_calibration.ready:
            reasons.append('IMU_NOT_CALIBRATED: ' + self.imu_calibration.detail)
        elif now - self.imu_last_time > cfg.imu_stale_s:
            reasons.append('IMU_STALE: no /livox/imu for %.2fs' % (now - self.imu_last_time))

        if self.initial_pose_time is None:
            reasons.append('INITIAL_POSE_REQUIRED: use RViz 2D Pose Estimate')
        elif self.localization_success_time is None:
            reasons.append('LOCALIZATION_NOT_CONFIRMED: waiting for relocation success log')
        elif self.localization_success_time < self.initial_pose_time:
            reasons.append('LOCALIZATION_PREDATES_INITIAL_POSE: reinitialize pose')

        if self.odom_xy is None:
            reasons.append('ODOM_REQUIRED: no valid localization odometry')
        elif now - self.odom_time > cfg.odom_stale_s:
            reasons.append('ODOM_STALE: no localization odometry for %.2fs' % (now - self.odom_time))
        elif (localization_confirmed and
              self.odom_time >= self.localization_success_time and
              not self.localization_anchor_valid and
              self.initial_pose_xy is not None):
            # Position/yaw matching is a one-time acquisition check.  It must
            # never be re-evaluated after motion starts, otherwise a robot that
            # legitimately travels more than 5 m would revoke its own motion.
            distance = math.dist(self.odom_xy, self.initial_pose_xy)
            yaw_error = 0.0
            if self.initial_pose_yaw is not None and self.odom_yaw is not None:
                yaw_error = abs(math.remainder(
                    self.odom_yaw - self.initial_pose_yaw, 2.0 * math.pi))
            if distance > cfg.max_pose_to_initial_pose_m:
                reasons.append(
                    'POSE_NOT_ANCHORED: odom is %.2fm from selected initial pose' % distance
                )
            elif yaw_error > cfg.max_yaw_to_initial_pose_rad:
                reasons.append(
                    'ORIENTATION_NOT_ANCHORED: odom yaw differs by %.2f rad'
                    % yaw_error)
            else:
                self.localization_anchor_valid = True
        return reasons

    def goal_commands_ready(self, now: float) -> bool:
        return not self._localization_reasons(now)

    def evaluate(self, now: float) -> Tuple[bool, List[str]]:
        cfg = self.config
        reasons = self._localization_reasons(now)
        if not reasons:
            if self.goal_time is None:
                reasons.append('GOAL_REQUIRED: no FAR Goalpoint in this localization epoch')
            elif self.waypoint_time is None:
                reasons.append('WAYPOINT_REQUIRED: FAR has not produced a waypoint for this goal')
            elif self.path_time is None:
                reasons.append('PATH_REQUIRED: localPlanner has not produced a new path')
            elif now - self.path_time > cfg.path_stale_s:
                reasons.append('PATH_STALE: no local path for %.2fs' % (now - self.path_time))
        self.last_rejection = reasons[0] if reasons else ''
        return not reasons, reasons

    def status(self, now: float) -> Dict[str, object]:
        motion_ready, reasons = self.evaluate(now)
        cal = self.imu_calibration
        return {
            'goal_commands_ready': self.goal_commands_ready(now),
            'motion_ready': motion_ready,
            'reasons': reasons,
            'imu': {
                'state': cal.state,
                'ready': cal.ready,
                'samples': cal.samples,
                'span_s': round(cal.span_s, 3),
                'gyro_bias': None if cal.gyro_bias is None else round(cal.gyro_bias, 6),
                'gyro_std': None if cal.gyro_std is None else round(cal.gyro_std, 6),
                'accel_norm': None if cal.accel_norm is None else round(cal.accel_norm, 4),
                'detail': cal.detail,
            },
            'initial_pose': None if self.initial_pose_xy is None else {
                'xy': [round(v, 4) for v in self.initial_pose_xy],
                'yaw': round(self.initial_pose_yaw, 5),
            },
            'localization_confirmed': self.localization_success_time is not None,
            'localization_anchor_valid': self.localization_anchor_valid,
            'odom_xy': None if self.odom_xy is None else {
                'xy': [round(v, 4) for v in self.odom_xy],
                'yaw': round(self.odom_yaw, 5),
            },
            'odom_age_s': None if not self.odom_time else round(now - self.odom_time, 3),
            'goal_xyz': None if self.goal_xyz is None else [round(v, 4) for v in self.goal_xyz],
            'waypoint_xyz': None if self.waypoint_xyz is None else [round(v, 4) for v in self.waypoint_xyz],
            'path_points': self.path_points,
            'path_age_s': None if self.path_time is None else round(now - self.path_time, 3),
        }


def decode_sport_request(api_id: int, parameter: str) -> Tuple[str, Optional[Tuple[float, float, float]]]:
    """Validate the only two pathFollower commands allowed through the gate."""
    if api_id == ROBOT_SPORT_API_ID_STOPMOVE:
        return 'stop', None
    if api_id != ROBOT_SPORT_API_ID_MOVE:
        raise ValueError('unsupported Sport API id %d; only StopMove/Move are allowed' % api_id)
    try:
        payload = json.loads(parameter or '{}')
    except (TypeError, ValueError) as error:
        raise ValueError('Move parameter is not valid JSON: %s' % error)
    if not isinstance(payload, dict):
        raise ValueError('Move parameter must be a JSON object')
    try:
        values = (payload['x'], payload['y'], payload['z'])
        values = tuple(float(v) for v in values)
    except (KeyError, TypeError, ValueError):
        raise ValueError('Move parameter must contain numeric x, y, z')
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Move parameter contains non-finite velocity')
    return 'move', values
