"""Loading MVSEC IMU and ground truth, and putting both in one gravity-aligned world frame."""

from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

IMU_TOPIC = "/davis/left/imu"
GROUND_TRUTH_TOPIC = "/davis/left/pose"
STANDARD_GRAVITY = 9.81


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bags(data_bag_path, ground_truth_bag_path):
    """Reads the left DAVIS IMU and the left camera ground-truth pose from the two MVSEC bags."""
    typestore = get_typestore(Stores.ROS1_NOETIC)

    imu_rows = []
    with AnyReader([Path(data_bag_path)], default_typestore=typestore) as reader:
        imu_connections = [c for c in reader.connections if c.topic == IMU_TOPIC]
        for connection, _, raw_message in reader.messages(connections=imu_connections):
            message = reader.deserialize(raw_message, connection.msgtype)
            accel = message.linear_acceleration
            gyro = message.angular_velocity
            imu_rows.append([
                stamp_to_seconds(message.header.stamp),
                accel.x, accel.y, accel.z,
                gyro.x, gyro.y, gyro.z,
            ])

    pose_rows = []
    with AnyReader([Path(ground_truth_bag_path)], default_typestore=typestore) as reader:
        pose_connections = [c for c in reader.connections if c.topic == GROUND_TRUTH_TOPIC]
        for connection, _, raw_message in reader.messages(connections=pose_connections):
            message = reader.deserialize(raw_message, connection.msgtype)
            position = message.pose.position
            quaternion = message.pose.orientation
            pose_rows.append([
                stamp_to_seconds(message.header.stamp),
                position.x, position.y, position.z,
                quaternion.x, quaternion.y, quaternion.z, quaternion.w,
            ])

    imu_rows = np.array(imu_rows)
    pose_rows = np.array(pose_rows)

    sequence = {
        "imu_time": imu_rows[:, 0],
        "accel": imu_rows[:, 1:4],
        "gyro": imu_rows[:, 4:7],
        "ground_truth_time": pose_rows[:, 0],
        "ground_truth_position": pose_rows[:, 1:4],
        "ground_truth_quaternion": pose_rows[:, 4:8],
    }
    return sequence


def load_sequence(data_bag_path, ground_truth_bag_path, cache_path):
    """Reading the bags takes a while, so the arrays are cached to an .npz file after the first run."""
    cache_path = Path(cache_path)
    if cache_path.exists():
        cached = np.load(cache_path)
        return {key: cached[key] for key in cached.files}

    sequence = read_bags(data_bag_path, ground_truth_bag_path)
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache_path, **sequence)
    return sequence


def load_imu_to_camera_extrinsics(calibration_path):
    """T_cam_imu maps a point from the IMU frame into the left camera frame."""
    with open(calibration_path) as calibration_file:
        calibration = yaml.safe_load(calibration_file)

    imu_to_camera = np.array(calibration["cam0"]["T_cam_imu"])
    camera_from_imu_rotation = imu_to_camera[:3, :3]
    imu_origin_in_camera = imu_to_camera[:3, 3]
    return camera_from_imu_rotation, imu_origin_in_camera


def estimate_static_biases(sequence, static_duration):
    """
    The drone sits on the floor for the first few seconds. During that window the
    gyro should read zero and the accelerometer should read gravity only.
    """
    time_since_start = sequence["imu_time"] - sequence["imu_time"][0]
    is_static = time_since_start < static_duration

    mean_accel = sequence["accel"][is_static].mean(axis=0)
    mean_gyro = sequence["gyro"][is_static].mean(axis=0)

    # At rest the accelerometer measures the floor pushing up, so the mean reading points up
    up_direction_in_imu = mean_accel / np.linalg.norm(mean_accel)

    # Only the bias along gravity is observable at rest. The sideways bias stays zero here.
    accel_bias = mean_accel - STANDARD_GRAVITY * up_direction_in_imu
    gyro_bias = mean_gyro

    static_result = {
        "accel_bias": accel_bias,
        "gyro_bias": gyro_bias,
        "up_direction_in_imu": up_direction_in_imu,
        "mean_accel_magnitude": np.linalg.norm(mean_accel),
        "accel_std_at_rest": sequence["accel"][is_static].std(axis=0),
    }
    return static_result


def build_world_frame(up_direction_in_imu, camera_from_imu_rotation):
    """
    MVSEC ground truth is expressed in the first camera frame (x right, y down, z forward).
    That frame is tilted about 17 degrees because the camera looks slightly down.

    The world frame used here is gravity-aligned:
        z points up,
        x is the camera's forward direction projected onto the floor,
        y points left.
    Yaw cannot be measured by an IMU, so pointing x along the initial heading is a choice, not an estimate.
    """
    up_in_first_camera = camera_from_imu_rotation @ up_direction_in_imu

    camera_forward = np.array([0.0, 0.0, 1.0])
    forward_on_floor = camera_forward - up_in_first_camera * np.dot(camera_forward, up_in_first_camera)
    forward_on_floor = forward_on_floor / np.linalg.norm(forward_on_floor)
    left = np.cross(up_in_first_camera, forward_on_floor)

    # Rows are the world axes written in first-camera coordinates
    world_from_first_camera = np.stack([forward_on_floor, left, up_in_first_camera])
    return world_from_first_camera


def estimate_ground_truth_time_offset(sequence, camera_from_imu_rotation, search_window=0.1, step=0.0025):
    """
    The motion-capture clock and the IMU clock are not perfectly in sync. The offset is found by
    sliding the ground-truth angular velocity over the gyro and keeping the shift that matches best.
    """
    ground_truth_time = sequence["ground_truth_time"]
    ground_truth_rotations = Rotation.from_quat(sequence["ground_truth_quaternion"])

    # Angular velocity from consecutive ground-truth orientations, in the camera frame
    time_steps = np.diff(ground_truth_time)
    relative_rotations = ground_truth_rotations[:-1].inv() * ground_truth_rotations[1:]
    ground_truth_angular_velocity = relative_rotations.as_rotvec() / time_steps[:, None]
    midpoint_times = (ground_truth_time[:-1] + ground_truth_time[1:]) / 2

    gyro_in_camera = sequence["gyro"] @ camera_from_imu_rotation.T

    best_offset = 0.0
    best_error = np.inf
    for offset in np.arange(-search_window, search_window + step / 2, step):
        gyro_at_ground_truth_times = np.zeros_like(ground_truth_angular_velocity)
        for axis in range(3):
            gyro_at_ground_truth_times[:, axis] = np.interp(
                midpoint_times + offset, sequence["imu_time"], gyro_in_camera[:, axis]
            )
        mean_error = np.mean(np.linalg.norm(gyro_at_ground_truth_times - ground_truth_angular_velocity, axis=1))
        if mean_error < best_error:
            best_error = mean_error
            best_offset = offset

    return best_offset


def ground_truth_imu_poses(sequence, world_from_first_camera, camera_from_imu_rotation, imu_origin_in_camera,
                           time_offset):
    """
    Converts the ground-truth camera poses into IMU poses in the world frame, so they can be
    compared one-to-one with the IMU estimate.
    """
    ground_truth_time = sequence["ground_truth_time"] + time_offset

    camera_positions_world = sequence["ground_truth_position"] @ world_from_first_camera.T
    world_from_camera = Rotation.from_matrix(world_from_first_camera) * Rotation.from_quat(
        sequence["ground_truth_quaternion"]
    )

    world_from_imu = world_from_camera * Rotation.from_matrix(camera_from_imu_rotation)
    # The IMU sits 1.8 cm from the camera centre, so the lever arm is small but still applied
    imu_positions_world = camera_positions_world + world_from_camera.apply(imu_origin_in_camera)

    return ground_truth_time, imu_positions_world, world_from_imu.as_matrix()
