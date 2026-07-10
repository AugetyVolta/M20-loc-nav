"""Small ROS-style quaternion helpers using the [x, y, z, w] convention."""

from __future__ import annotations

import math

import numpy as np


def quaternion_matrix(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    norm = x * x + y * y + z * z + w * w
    matrix = np.identity(4, dtype=np.float64)
    if norm < np.finfo(float).eps:
        return matrix

    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale

    matrix[:3, :3] = (
        (1.0 - yy - zz, xy - wz, xz + wy),
        (xy + wz, 1.0 - xx - zz, yz - wx),
        (xz - wy, yz + wx, 1.0 - xx - yy),
    )
    return matrix


def quaternion_multiply(quaternion1, quaternion0):
    x1, y1, z1, w1 = (float(value) for value in quaternion1)
    x0, y0, z0, w0 = (float(value) for value in quaternion0)
    return np.array(
        (
            w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
            w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0,
            w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0,
            w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0,
        ),
        dtype=np.float64,
    )


def quaternion_inverse(quaternion):
    values = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.dot(values, values))
    if norm < np.finfo(float).eps:
        return np.zeros(4, dtype=np.float64)
    return np.array((-values[0], -values[1], -values[2], values[3])) / norm


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = 0.5 * float(roll)
    half_pitch = 0.5 * float(pitch)
    half_yaw = 0.5 * float(yaw)

    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return np.array(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ),
        dtype=np.float64,
    )


def euler_from_quaternion(quaternion):
    x, y, z, w = (float(value) for value in quaternion)

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw
