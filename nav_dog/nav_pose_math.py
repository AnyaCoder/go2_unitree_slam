#!/usr/bin/env python3
"""Small dependency-free rigid-transform helpers for the nav pose bridge."""

import math
from collections import namedtuple


Pose = namedtuple('Pose', 'x y z qx qy qz qw')
IDENTITY_POSE = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def normalize_quaternion(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    return (x / norm, y / norm, z / norm, w / norm)


def multiply_quaternions(left, right):
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def conjugate_quaternion(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return (-x, -y, -z, w)


def rotate_vector(quaternion, vector):
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = (float(value) for value in vector)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose(left, right):
    """Return left * right, where a pose maps its child frame into its parent."""
    rotated = rotate_vector(left[3:], right[:3])
    translation = tuple(left[i] + rotated[i] for i in range(3))
    rotation = multiply_quaternions(left[3:], right[3:])
    return Pose(*(translation + rotation))


def inverse(pose):
    inverse_rotation = conjugate_quaternion(pose[3:])
    translated = rotate_vector(inverse_rotation, (-pose[0], -pose[1], -pose[2]))
    return Pose(*(translated + inverse_rotation))


def correction_from_global(global_pose, local_pose):
    """Return map<-local correction C such that global = C * local."""
    return compose(global_pose, inverse(local_pose))


def slerp(left, right, alpha):
    left = normalize_quaternion(left)
    right = normalize_quaternion(right)
    dot = sum(a * b for a, b in zip(left, right))
    if dot < 0.0:
        right = tuple(-value for value in right)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        blended = tuple(a + alpha * (b - a) for a, b in zip(left, right))
        return normalize_quaternion(blended)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    left_scale = math.sin((1.0 - alpha) * theta) / sin_theta
    right_scale = math.sin(alpha * theta) / sin_theta
    return normalize_quaternion(tuple(
        left_scale * a + right_scale * b for a, b in zip(left, right)))


def blend_pose(left, right, alpha):
    alpha = max(0.0, min(1.0, float(alpha)))
    translation = tuple(
        left[i] + alpha * (right[i] - left[i]) for i in range(3))
    rotation = slerp(left[3:], right[3:], alpha)
    return Pose(*(translation + rotation))


def pose_distance(left, right):
    return math.sqrt(sum((left[i] - right[i]) ** 2 for i in range(3)))
