"""Strapdown integration: turning gyro and accelerometer samples into orientation, velocity, and position."""

import numpy as np

GRAVITY_WORLD = np.array([0.0, 0.0, -9.81])


def skew(vector):
    """Matrix form of the cross product, so that skew(a) @ b == np.cross(a, b)."""
    return np.array([
        [0.0, -vector[2], vector[1]],
        [vector[2], 0.0, -vector[0]],
        [-vector[1], vector[0], 0.0],
    ])


def rotation_from_rotvec(rotation_vector):
    """
    Rodrigues' formula. Written out by hand because it runs 20,000 times per sequence and
    scipy's Rotation object is slow when called one sample at a time.
    """
    angle = np.linalg.norm(rotation_vector)
    if angle < 1e-12:
        return np.eye(3) + skew(rotation_vector)

    axis = rotation_vector / angle
    axis_skew = skew(axis)
    return np.eye(3) + np.sin(angle) * axis_skew + (1.0 - np.cos(angle)) * (axis_skew @ axis_skew)


def integrate_gyro(imu_time, gyro, gyro_bias, initial_rotation):
    """
    Orientation from the gyro alone. Each sample rotates the body by (angular rate * dt),
    applied on the right because the gyro measures rotation in the body's own frame.
    """
    num_samples = len(imu_time)
    rotations = np.zeros((num_samples, 3, 3))
    rotations[0] = initial_rotation

    for index in range(1, num_samples):
        dt = imu_time[index] - imu_time[index - 1]
        angular_rate = gyro[index - 1] - gyro_bias
        rotations[index] = rotations[index - 1] @ rotation_from_rotvec(angular_rate * dt)

    return rotations


def dead_reckoning(imu_time, accel, gyro, accel_bias, gyro_bias, initial_rotation, initial_position):
    """
    IMU-only position: rotate each accelerometer sample into the world frame, add gravity back,
    and integrate twice. Any small error in acceleration grows quadratically in position.
    """
    num_samples = len(imu_time)
    positions = np.zeros((num_samples, 3))
    rotations = np.zeros((num_samples, 3, 3))

    position = initial_position.copy()
    velocity = np.zeros(3)
    rotation = initial_rotation.copy()
    positions[0] = position
    rotations[0] = rotation

    for index in range(1, num_samples):
        dt = imu_time[index] - imu_time[index - 1]
        specific_force = accel[index - 1] - accel_bias
        angular_rate = gyro[index - 1] - gyro_bias

        # The accelerometer feels specific force (acceleration minus gravity), so gravity is added back
        acceleration_world = rotation @ specific_force + GRAVITY_WORLD

        position = position + velocity * dt + 0.5 * acceleration_world * dt**2
        velocity = velocity + acceleration_world * dt
        rotation = rotation @ rotation_from_rotvec(angular_rate * dt)

        positions[index] = position
        rotations[index] = rotation

    return positions, rotations
