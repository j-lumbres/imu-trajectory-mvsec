"""Comparing an estimate with ground truth at the ground-truth timestamps."""

import numpy as np
from scipy.spatial.transform import Rotation


def nearest_indices(source_time, query_time):
    """The IMU runs at 1 kHz, so the nearest sample is at most 0.5 ms away from any query time."""
    indices = np.searchsorted(source_time, query_time)
    indices = np.clip(indices, 1, len(source_time) - 1)
    previous_is_closer = (query_time - source_time[indices - 1]) < (source_time[indices] - query_time)
    indices[previous_is_closer] -= 1
    return indices


def position_errors(estimated_positions, ground_truth_positions):
    return np.linalg.norm(estimated_positions - ground_truth_positions, axis=1)


def rotation_errors_degrees(estimated_rotations, ground_truth_rotations):
    """Angle of the rotation that takes the ground-truth orientation to the estimated one."""
    relative = Rotation.from_matrix(ground_truth_rotations).inv() * Rotation.from_matrix(estimated_rotations)
    return np.degrees(relative.magnitude())


def body_euler_degrees(world_from_imu_rotations, imu_from_body_rotation):
    """
    The IMU axes follow the camera (x right, y down, z forward), which makes roll, pitch, and yaw
    hard to read. The body frame is the IMU frame re-oriented so that it starts level with
    x forward, y left, z up. Yaw, pitch, and roll then use the usual aircraft order (Z, Y, X).
    """
    world_from_body = Rotation.from_matrix(world_from_imu_rotations) * Rotation.from_matrix(imu_from_body_rotation)
    yaw_pitch_roll = world_from_body.as_euler("ZYX", degrees=True)
    return yaw_pitch_roll


def summarize(name, position_error, rotation_error):
    position_rmse = np.sqrt(np.mean(position_error**2))
    print(f"{name:28s} position RMSE {position_rmse:6.3f} m | max {position_error.max():6.2f} m | "
          f"rotation mean {rotation_error.mean():5.2f} deg | max {rotation_error.max():5.2f} deg")
