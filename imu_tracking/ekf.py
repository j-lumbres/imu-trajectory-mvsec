"""
Error-state Kalman filter that fuses the IMU with sparse position fixes.

The IMU is integrated at 1 kHz exactly as in dead reckoning. Once per fix interval, a position
measurement pulls the estimate back and, through the covariance, also corrects velocity,
attitude, and the sensor biases.

Error state (15 numbers):
    [0:3]   position error
    [3:6]   velocity error
    [6:9]   attitude error, a small rotation in the body frame
    [9:12]  accelerometer bias error
    [12:15] gyro bias error
"""

import numpy as np

from imu_tracking.integration import GRAVITY_WORLD, rotation_from_rotvec, skew


def run_position_aided_ekf(imu_time, accel, gyro, fix_time, fix_position,
                           initial_rotation, initial_position, initial_accel_bias, initial_gyro_bias,
                           noise):
    num_samples = len(imu_time)
    positions = np.zeros((num_samples, 3))
    rotations = np.zeros((num_samples, 3, 3))
    accel_biases = np.zeros((num_samples, 3))
    gyro_biases = np.zeros((num_samples, 3))

    position = initial_position.copy()
    velocity = np.zeros(3)
    rotation = initial_rotation.copy()
    accel_bias = initial_accel_bias.copy()
    gyro_bias = initial_gyro_bias.copy()

    covariance = np.diag(
        [noise["initial_position_std"] ** 2] * 3
        + [noise["initial_velocity_std"] ** 2] * 3
        + [noise["initial_attitude_std"] ** 2] * 3
        + [noise["initial_accel_bias_std"] ** 2] * 3
        + [noise["initial_gyro_bias_std"] ** 2] * 3
    )
    measurement_jacobian = np.zeros((3, 15))
    measurement_jacobian[:, 0:3] = np.eye(3)
    measurement_covariance = np.eye(3) * noise["fix_position_std"] ** 2

    positions[0] = position
    rotations[0] = rotation
    accel_biases[0] = accel_bias
    gyro_biases[0] = gyro_bias
    next_fix_index = 0
    used_fix_indices = []

    for index in range(1, num_samples):
        dt = imu_time[index] - imu_time[index - 1]
        specific_force = accel[index - 1] - accel_bias
        angular_rate = gyro[index - 1] - gyro_bias

        # Prediction: the same strapdown integration as dead reckoning
        acceleration_world = rotation @ specific_force + GRAVITY_WORLD
        position = position + velocity * dt + 0.5 * acceleration_world * dt**2
        velocity = velocity + acceleration_world * dt
        step_rotation = rotation_from_rotvec(angular_rate * dt)
        rotation_before_step = rotation
        rotation = rotation @ step_rotation

        # How a small error in each state grows over one step (linearised error dynamics)
        transition = np.eye(15)
        transition[0:3, 3:6] = np.eye(3) * dt
        transition[3:6, 6:9] = -rotation_before_step @ skew(specific_force) * dt
        transition[3:6, 9:12] = -rotation_before_step * dt
        transition[6:9, 6:9] = step_rotation.T
        transition[6:9, 12:15] = -np.eye(3) * dt

        process_noise = np.diag(
            [0.0] * 3
            + [(noise["accel_std"] * dt) ** 2] * 3
            + [(noise["gyro_std"] * dt) ** 2] * 3
            + [noise["accel_bias_walk"] ** 2 * dt] * 3
            + [noise["gyro_bias_walk"] ** 2 * dt] * 3
        )
        covariance = transition @ covariance @ transition.T + process_noise

        # Correction: only when the next position fix has arrived
        fix_is_due = next_fix_index < len(fix_time) and imu_time[index] >= fix_time[next_fix_index]
        if fix_is_due:
            residual = fix_position[next_fix_index] - position

            innovation_covariance = (measurement_jacobian @ covariance @ measurement_jacobian.T
                                     + measurement_covariance)
            kalman_gain = covariance @ measurement_jacobian.T @ np.linalg.inv(innovation_covariance)
            error_estimate = kalman_gain @ residual

            position = position + error_estimate[0:3]
            velocity = velocity + error_estimate[3:6]
            rotation = rotation @ rotation_from_rotvec(error_estimate[6:9])
            accel_bias = accel_bias + error_estimate[9:12]
            gyro_bias = gyro_bias + error_estimate[12:15]

            # Joseph form keeps the covariance symmetric and positive definite
            identity_minus_gain = np.eye(15) - kalman_gain @ measurement_jacobian
            covariance = (identity_minus_gain @ covariance @ identity_minus_gain.T
                          + kalman_gain @ measurement_covariance @ kalman_gain.T)

            used_fix_indices.append(next_fix_index)
            next_fix_index += 1

        positions[index] = position
        rotations[index] = rotation
        accel_biases[index] = accel_bias
        gyro_biases[index] = gyro_bias

    result = {
        "positions": positions,
        "rotations": rotations,
        "accel_biases": accel_biases,
        "gyro_biases": gyro_biases,
        "used_fix_indices": np.array(used_fix_indices),
    }
    return result
