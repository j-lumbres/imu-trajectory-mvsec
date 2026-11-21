"""
Stereo visual odometry from the DAVIS grayscale frames.

The two DAVIS cameras sit 10 cm apart. Matching a corner between the left and right image gives
its disparity, and depth = focal_length * baseline / disparity, so every tracked point has a depth
in metres. Tracking those 3D points into the next left frame and solving Perspective-n-Point (PnP)
gives the camera motion between the two frames. Chaining the motions gives the trajectory.
Nothing here reads the ground truth.
"""

from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

LEFT_IMAGE_TOPIC = "/davis/left/image_raw"
RIGHT_IMAGE_TOPIC = "/davis/right/image_raw"
IMAGE_SIZE = (346, 260)  # width, height


def load_stereo_frames(data_bag_path, cache_path):
    """Reads both grayscale streams once and caches them, because the bag also holds events and LiDAR."""
    cache_path = Path(cache_path)
    if cache_path.exists():
        cached = np.load(cache_path)
        return {key: cached[key] for key in cached.files}

    frames = {LEFT_IMAGE_TOPIC: [], RIGHT_IMAGE_TOPIC: []}
    times = {LEFT_IMAGE_TOPIC: [], RIGHT_IMAGE_TOPIC: []}
    typestore = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([Path(data_bag_path)], default_typestore=typestore) as reader:
        image_connections = [c for c in reader.connections if c.topic in frames]
        for connection, _, raw_message in reader.messages(connections=image_connections):
            message = reader.deserialize(raw_message, connection.msgtype)
            image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width)
            frames[connection.topic].append(image.copy())
            times[connection.topic].append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    stereo_frames = {
        "left_time": np.array(times[LEFT_IMAGE_TOPIC]),
        "left_images": np.stack(frames[LEFT_IMAGE_TOPIC]),
        "right_time": np.array(times[RIGHT_IMAGE_TOPIC]),
        "right_images": np.stack(frames[RIGHT_IMAGE_TOPIC]),
    }
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache_path, **stereo_frames)
    return stereo_frames


def load_stereo_rectification(calibration_path):
    """
    The MVSEC calibration already gives rectification rotations and projection matrices. After
    rectification both images look like pinhole cameras with parallel optical axes, so a point
    in the left image lies on the same row in the right image.
    """
    with open(calibration_path) as calibration_file:
        calibration = yaml.safe_load(calibration_file)

    rectification = {}
    for camera_name in ["cam0", "cam1"]:
        camera = calibration[camera_name]
        intrinsics = camera["intrinsics"]
        camera_matrix = np.array([
            [intrinsics[0], 0.0, intrinsics[2]],
            [0.0, intrinsics[1], intrinsics[3]],
            [0.0, 0.0, 1.0],
        ])
        # The DAVIS lenses follow the equidistant (fisheye) distortion model
        distortion = np.array(camera["distortion_coeffs"]).reshape(4, 1)
        rectification_rotation = np.array(camera["rectification_matrix"])
        projection_matrix = np.array(camera["projection_matrix"])

        map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
            camera_matrix, distortion, rectification_rotation, projection_matrix[:, :3], IMAGE_SIZE, cv2.CV_32FC1
        )
        rectification[camera_name] = {
            "map_x": map_x,
            "map_y": map_y,
            "rectification_rotation": rectification_rotation,
            "projection_matrix": projection_matrix,
        }

    left_projection = rectification["cam0"]["projection_matrix"]
    right_projection = rectification["cam1"]["projection_matrix"]
    rectification["focal_length"] = left_projection[0, 0]
    rectification["principal_point_x"] = left_projection[0, 2]
    rectification["principal_point_y"] = left_projection[1, 2]
    # The right projection matrix stores -focal_length * baseline in its last column
    rectification["baseline"] = -right_projection[0, 3] / right_projection[0, 0]
    return rectification


def rectify_image(image, camera_rectification, contrast_enhancer):
    rectified = cv2.remap(image, camera_rectification["map_x"], camera_rectification["map_y"], cv2.INTER_LINEAR)
    # The indoor frames are dark (mean brightness about 31 of 255), so local contrast is boosted before tracking
    return contrast_enhancer.apply(rectified)


def nearest_right_frames(left_time, right_time):
    indices = np.searchsorted(right_time, left_time)
    indices = np.clip(indices, 1, len(right_time) - 1)
    previous_is_closer = np.abs(right_time[indices - 1] - left_time) < np.abs(right_time[indices] - left_time)
    indices[previous_is_closer] -= 1
    return indices


def triangulate_stereo_corners(left_image, right_image, rectification, settings):
    """Detects corners in the left image, finds each one in the right image, and turns disparity into depth."""
    corners = cv2.goodFeaturesToTrack(
        left_image, settings["max_corners"], settings["corner_quality"], settings["min_corner_distance"]
    )
    if corners is None:
        return None, None

    matched, found, _ = cv2.calcOpticalFlowPyrLK(left_image, right_image, corners, None, **settings["optical_flow"])
    corners = corners.reshape(-1, 2)
    matched = matched.reshape(-1, 2)
    found = found.ravel() == 1

    disparity = corners[:, 0] - matched[:, 0]
    row_difference = np.abs(corners[:, 1] - matched[:, 1])
    # After rectification a correct match stays on the same row and lies to the left in the right image
    is_valid = found & (row_difference < settings["max_row_difference"]) & (disparity > settings["min_disparity"])
    corners = corners[is_valid]
    disparity = disparity[is_valid]

    focal_length = rectification["focal_length"]
    depth = focal_length * rectification["baseline"] / disparity
    points_x = (corners[:, 0] - rectification["principal_point_x"]) * depth / focal_length
    points_y = (corners[:, 1] - rectification["principal_point_y"]) * depth / focal_length
    points_3d = np.stack([points_x, points_y, depth], axis=1)
    return corners, points_3d


def run_stereo_vo(stereo_frames, rectification, settings):
    """
    Frame-to-frame stereo VO. Returns the left camera pose at every left frame, written in the
    frame of the first left camera (unrectified, x right, y down, z forward), the same frame
    MVSEC uses for its ground truth.
    """
    contrast_enhancer = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    focal_length = rectification["focal_length"]
    rectified_camera_matrix = np.array([
        [focal_length, 0.0, rectification["principal_point_x"]],
        [0.0, focal_length, rectification["principal_point_y"]],
        [0.0, 0.0, 1.0],
    ])

    left_time = stereo_frames["left_time"]
    right_indices = nearest_right_frames(left_time, stereo_frames["right_time"])
    num_frames = len(left_time)

    previous_left = rectify_image(stereo_frames["left_images"][0], rectification["cam0"], contrast_enhancer)
    previous_right = rectify_image(stereo_frames["right_images"][right_indices[0]], rectification["cam1"], contrast_enhancer)

    rectified_poses = np.zeros((num_frames, 4, 4))
    rectified_poses[0] = np.eye(4)
    inlier_counts = np.zeros(num_frames, dtype=int)
    current_pose = np.eye(4)

    for frame_index in range(1, num_frames):
        current_left = rectify_image(stereo_frames["left_images"][frame_index], rectification["cam0"], contrast_enhancer)
        current_right = rectify_image(
            stereo_frames["right_images"][right_indices[frame_index]], rectification["cam1"], contrast_enhancer
        )

        # If PnP fails the motion is assumed to be zero for this frame. It never failed on indoor_flying4.
        relative_motion = np.eye(4)
        corners, points_3d = triangulate_stereo_corners(previous_left, previous_right, rectification, settings)

        if corners is not None and len(corners) >= settings["min_points"]:
            tracked, found, _ = cv2.calcOpticalFlowPyrLK(
                previous_left, current_left, corners.reshape(-1, 1, 2).astype(np.float32), None,
                **settings["optical_flow"]
            )
            found = found.ravel() == 1

            if found.sum() >= settings["min_points"]:
                success, rotation_vector, translation, inliers = cv2.solvePnPRansac(
                    points_3d[found], tracked.reshape(-1, 2)[found], rectified_camera_matrix, None,
                    reprojectionError=settings["pnp_reprojection_error"], iterationsCount=settings["pnp_iterations"],
                )
                if success and inliers is not None and len(inliers) >= settings["min_points"]:
                    # PnP returns the transform that maps previous-camera points into the current camera
                    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
                    previous_to_current = np.eye(4)
                    previous_to_current[:3, :3] = rotation_matrix
                    previous_to_current[:3, 3] = translation.ravel()
                    relative_motion = np.linalg.inv(previous_to_current)
                    inlier_counts[frame_index] = len(inliers)

        current_pose = current_pose @ relative_motion
        rectified_poses[frame_index] = current_pose
        previous_left = current_left
        previous_right = current_right

    # The poses above live in the rectified left frame. Undo the small rectification rotation
    # so they are expressed in the raw left camera frame, like the ground truth.
    rectified_from_raw = np.eye(4)
    rectified_from_raw[:3, :3] = rectification["cam0"]["rectification_rotation"]
    camera_poses = np.zeros_like(rectified_poses)
    for frame_index in range(num_frames):
        camera_poses[frame_index] = rectified_from_raw.T @ rectified_poses[frame_index] @ rectified_from_raw

    return camera_poses, inlier_counts


def camera_poses_to_world_imu(camera_poses, world_from_first_camera, camera_from_imu_rotation, imu_origin_in_camera):
    """Puts the VO camera poses in the gravity-aligned world frame and moves them from the camera to the IMU."""
    camera_positions_world = camera_poses[:, :3, 3] @ world_from_first_camera.T
    world_from_camera = world_from_first_camera @ camera_poses[:, :3, :3]

    world_from_imu = world_from_camera @ camera_from_imu_rotation
    imu_positions_world = camera_positions_world + world_from_camera @ imu_origin_in_camera
    return imu_positions_world, world_from_imu

