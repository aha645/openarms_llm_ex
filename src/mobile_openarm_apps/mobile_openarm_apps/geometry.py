"""ROS 비의존 기하 유틸 (numpy 만 사용 → 단위 테스트 가능)."""
import math

import numpy as np


def yaw_to_quat(yaw):
    """(x, y, z, w)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def mat_to_quat_xyzw(R):
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def rot_axis(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def approach_rotation(pitch_deg, roll_deg=0.0, forward=(1.0, 0.0, 0.0)):
    """base_footprint 기준 그리퍼(TCP) 자세.

    TCP z축 = 접근 방향: 전방 수평에서 아래로 pitch_deg 만큼 숙임.
    roll_deg: 접근축(z) 둘레 회전 — 손가락 벌림 방향 선택 (IK 가 안 풀리면 여러 값을 시도).
    """
    f = np.asarray(forward, float)
    f /= np.linalg.norm(f)
    a = math.radians(pitch_deg)
    z = f * math.cos(a) + np.array([0, 0, -1.0]) * math.sin(a)
    x = np.cross([0, 0, 1.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(f, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    return rot_axis(z, math.radians(roll_deg)) @ R


def se2_apply(pose_xyyaw, pt_xy):
    x, y, th = pose_xyyaw
    c, s = math.cos(th), math.sin(th)
    return np.array([x + c * pt_xy[0] - s * pt_xy[1], y + s * pt_xy[0] + c * pt_xy[1]])


def se2_inverse_apply(pose_xyyaw, pt_xy):
    """map 좌표 점 → 로봇(base_footprint) 좌표."""
    x, y, th = pose_xyyaw
    dx, dy = pt_xy[0] - x, pt_xy[1] - y
    c, s = math.cos(th), math.sin(th)
    return np.array([c * dx + s * dy, -s * dx + c * dy])
