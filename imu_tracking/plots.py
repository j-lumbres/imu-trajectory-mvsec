"""
Figures in the style used by IMU and VIO papers: estimate and ground truth overlaid on the same
axes, per-axis time series, an error panel, and a 6-DoF animation of the camera pose.
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

GROUND_TRUTH_COLOR = "black"
ESTIMATE_COLORS = {
    "Gyro only": "#d1495b",
    "IMU only (dead reckoning)": "#d1495b",
    "EKF (IMU + 1 Hz position fix)": "#1f5aa6",
    "Stereo VO alone": "#7a4fa3",
    "EKF (IMU + stereo VO)": "#1f5aa6",
}
FIX_COLOR = "#edae49"
AXIS_COLORS = ["#d1495b", "#3a9d23", "#1f5aa6"]

# The frustum uses the DAVIS 346 field of view: half the image width (173 px) and height (130 px)
# divided by the focal length (226 px). Its depth is only a drawing size.
FRUSTUM_DEPTH = 0.35  # metres
FRUSTUM_HALF_WIDTH = FRUSTUM_DEPTH * 173.0 / 226.0
FRUSTUM_HALF_HEIGHT = FRUSTUM_DEPTH * 130.0 / 226.0


def use_paper_style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "legend.fontsize": 8,
        "legend.frameon": False,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "lines.linewidth": 1.2,
        "figure.dpi": 120,
        "savefig.dpi": 200,
        "savefig.bbox": "tight",
    })


def color_for(name):
    return ESTIMATE_COLORS.get(name, "#1f5aa6")


def plot_raw_imu(time, accel, gyro, static_duration):
    figure, (accel_axis, gyro_axis) = plt.subplots(2, 1, figsize=(7, 4.2), sharex=True)

    for axis_index, axis_name in enumerate(["x", "y", "z"]):
        accel_axis.plot(time, accel[:, axis_index], color=AXIS_COLORS[axis_index], linewidth=0.6, label=axis_name)
        gyro_axis.plot(time, gyro[:, axis_index], color=AXIS_COLORS[axis_index], linewidth=0.6, label=axis_name)

    for axis in (accel_axis, gyro_axis):
        axis.axvspan(0, static_duration, color="grey", alpha=0.15, label="static (bias window)")

    accel_axis.set_ylabel("Accelerometer (m/s$^2$)")
    gyro_axis.set_ylabel("Gyroscope (rad/s)")
    gyro_axis.set_xlabel("Time (s)")
    accel_axis.set_title("Raw IMU measurements, IMU frame")
    accel_axis.legend(ncol=4, loc="upper left")
    figure.align_ylabels()
    return figure


def plot_orientation(ground_truth_time, ground_truth_ypr, estimates):
    """estimates maps a name to (time, yaw_pitch_roll, rotation_error_at_ground_truth_times)."""
    figure, axes = plt.subplots(4, 1, figsize=(7, 7), sharex=True)
    angle_names = ["Yaw (deg)", "Pitch (deg)", "Roll (deg)"]

    for angle_index in range(3):
        axis = axes[angle_index]
        ground_truth_angle = ground_truth_ypr[:, angle_index]
        if angle_index == 0:
            ground_truth_angle = np.degrees(np.unwrap(np.radians(ground_truth_angle)))
        axis.plot(ground_truth_time, ground_truth_angle, color=GROUND_TRUTH_COLOR, linestyle="--", label="Ground truth")

        for name, (time, yaw_pitch_roll, _) in estimates.items():
            estimated_angle = yaw_pitch_roll[:, angle_index]
            if angle_index == 0:
                estimated_angle = np.degrees(np.unwrap(np.radians(estimated_angle)))
            axis.plot(time, estimated_angle, color=color_for(name), label=name)

        axis.set_ylabel(angle_names[angle_index])

    for name, (_, _, rotation_error) in estimates.items():
        axes[3].plot(ground_truth_time, rotation_error, color=color_for(name), label=name)
    axes[3].set_ylabel("Rotation error (deg)")
    axes[3].set_xlabel("Time (s)")

    axes[0].set_title("Orientation of the hexacopter body, world frame")
    axes[0].legend(loc="upper left")
    figure.align_ylabels()
    return figure


def plot_position_components(ground_truth_time, ground_truth_positions, estimates, fix_times=None, error_log_scale=False):
    """estimates maps a name to (time, positions, position_error_at_ground_truth_times)."""
    figure, axes = plt.subplots(4, 1, figsize=(7, 7), sharex=True)
    axis_names = ["x (m)", "y (m)", "z (m)"]

    for axis_index in range(3):
        axis = axes[axis_index]
        axis.plot(ground_truth_time, ground_truth_positions[:, axis_index],
                  color=GROUND_TRUTH_COLOR, linestyle="--", label="Ground truth")
        for name, (time, positions, _) in estimates.items():
            axis.plot(time, positions[:, axis_index], color=color_for(name), label=name)
        axis.set_ylabel(axis_names[axis_index])

        # Keep the panel readable when dead reckoning drifts far away from the flight volume
        ground_truth_low = ground_truth_positions[:, axis_index].min()
        ground_truth_high = ground_truth_positions[:, axis_index].max()
        margin = 0.5 * (ground_truth_high - ground_truth_low) + 0.3
        axis.set_ylim(ground_truth_low - margin, ground_truth_high + margin)

    for name, (_, _, position_error) in estimates.items():
        axes[3].plot(ground_truth_time, position_error, color=color_for(name), label=name)
    if fix_times is not None:
        for fix_time in fix_times:
            axes[3].axvline(fix_time, color=FIX_COLOR, linewidth=0.5, alpha=0.6)
    if error_log_scale:
        axes[3].set_yscale("log")
    axes[3].set_ylabel("Position error (m)")
    axes[3].set_xlabel("Time (s)")

    axes[0].set_title("Position, world frame (z up)")
    axes[0].legend(loc="upper left")
    figure.align_ylabels()
    return figure


def set_true_scale_3d_limits(axis, points, margin=0.3):
    """One metre has the same length on every axis, so a flat flight looks flat."""
    lower = points.min(axis=0) - margin
    upper = points.max(axis=0) + margin
    lower[2] = min(lower[2], -0.05)
    axis.set_xlim(lower[0], upper[0])
    axis.set_ylim(lower[1], upper[1])
    axis.set_zlim(lower[2], upper[2])
    axis.set_box_aspect(upper - lower)


def plot_trajectory(ground_truth_positions, estimates, fix_positions=None, title="Trajectory"):
    """estimates maps a name to an (N, 3) array of positions. Left panel is 3D, right panel is the top view."""
    figure = plt.figure(figsize=(11, 4.8))
    axis_3d = figure.add_subplot(1, 2, 1, projection="3d")
    axis_top = figure.add_subplot(1, 2, 2)
    figure.subplots_adjust(wspace=0.35)

    axis_3d.plot(*ground_truth_positions.T, color=GROUND_TRUTH_COLOR, linestyle="--", label="Ground truth")
    axis_top.plot(ground_truth_positions[:, 0], ground_truth_positions[:, 1],
                  color=GROUND_TRUTH_COLOR, linestyle="--", label="Ground truth")

    for name, positions in estimates.items():
        axis_3d.plot(*positions.T, color=color_for(name), label=name)
        axis_top.plot(positions[:, 0], positions[:, 1], color=color_for(name), label=name)

    if fix_positions is not None:
        axis_3d.scatter(*fix_positions.T, color=FIX_COLOR, s=10, label="Position fixes", depthshade=False)
        axis_top.scatter(fix_positions[:, 0], fix_positions[:, 1], color=FIX_COLOR, s=10, zorder=3,
                         label="Position fixes")

    start = ground_truth_positions[0]
    axis_3d.scatter(*start, color="black", marker="o", s=30)
    axis_top.scatter(start[0], start[1], color="black", marker="o", s=30, zorder=4, label="Start")

    set_true_scale_3d_limits(axis_3d, ground_truth_positions)
    axis_3d.set_xlabel("x (m)")
    axis_3d.set_ylabel("y (m)")
    axis_3d.set_zlabel("z (m)", labelpad=2)
    axis_3d.view_init(elev=25, azim=-60)
    axis_3d.set_title("3D view")

    axis_top.set_aspect("equal", adjustable="datalim")
    axis_top.set_xlabel("x, forward (m)")
    axis_top.set_ylabel("y, left (m)")
    axis_top.set_title("Top view (xy)")
    axis_top.legend(loc="best")

    figure.suptitle(title)
    return figure


def camera_frustum_faces(position, camera_rotation):
    """
    A camera frustum is the pyramid of space the camera can see: the apex is the camera centre and
    the base is the image rectangle pushed out along the viewing direction. Corners are written in
    the camera frame (x right, y down, z forward) and then rotated into the world.
    A small triangle on the top edge marks which way is up in the image.
    """
    corners_camera = np.array([
        [-FRUSTUM_HALF_WIDTH, -FRUSTUM_HALF_HEIGHT, FRUSTUM_DEPTH],  # top left
        [FRUSTUM_HALF_WIDTH, -FRUSTUM_HALF_HEIGHT, FRUSTUM_DEPTH],   # top right
        [FRUSTUM_HALF_WIDTH, FRUSTUM_HALF_HEIGHT, FRUSTUM_DEPTH],    # bottom right
        [-FRUSTUM_HALF_WIDTH, FRUSTUM_HALF_HEIGHT, FRUSTUM_DEPTH],   # bottom left
    ])
    up_marker_camera = np.array([
        [-0.4 * FRUSTUM_HALF_WIDTH, -FRUSTUM_HALF_HEIGHT, FRUSTUM_DEPTH],
        [0.4 * FRUSTUM_HALF_WIDTH, -FRUSTUM_HALF_HEIGHT, FRUSTUM_DEPTH],
        [0.0, -1.6 * FRUSTUM_HALF_HEIGHT, FRUSTUM_DEPTH],
    ])

    corners_world = position + corners_camera @ camera_rotation.T
    up_marker_world = position + up_marker_camera @ camera_rotation.T

    side_faces = []
    for corner_index in range(4):
        next_index = (corner_index + 1) % 4
        side_faces.append([position, corners_world[corner_index], corners_world[next_index]])
    image_plane = [list(corners_world)]
    up_marker = [list(up_marker_world)]
    return side_faces, image_plane, up_marker


def animate_flight(time, positions, camera_rotations, ground_truth_positions, ground_truth_camera_rotations,
                   output_path, estimate_name="Estimate", frames_per_second=15, trail_seconds=None, dpi=90):
    """
    A 6-DoF animation in the style of Madgwick's MATLAB SixDofAnimation. The estimated camera is a
    filled frustum that points where the camera looks, the ground-truth camera is a dashed outline,
    and both leave a trail. All inputs must already be sampled at the animation frame times.
    """
    estimate_color = color_for(estimate_name)

    figure = plt.figure(figsize=(7.5, 5.2))
    axis = figure.add_subplot(1, 1, 1, projection="3d")
    set_true_scale_3d_limits(axis, ground_truth_positions, margin=0.6)
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.view_init(elev=30, azim=-60)

    ground_truth_trail, = axis.plot([], [], [], color=GROUND_TRUTH_COLOR, linestyle="--", linewidth=1, label="Ground truth")
    estimate_trail, = axis.plot([], [], [], color=estimate_color, linewidth=1.4, label=estimate_name)

    # Outline only, so the ground truth stays visible when the two frustums overlap
    ground_truth_sides = Poly3DCollection([], facecolor="none", edgecolor="0.25", linewidths=0.9, linestyles="--")
    ground_truth_plane = Poly3DCollection([], facecolor="none", edgecolor="0.25", linewidths=0.9, linestyles="--")
    estimate_sides = Poly3DCollection([], facecolor=estimate_color, edgecolor=estimate_color, linewidths=0.8, alpha=0.25)
    estimate_plane = Poly3DCollection([], facecolor=estimate_color, edgecolor="black", linewidths=1.0, alpha=0.45)
    estimate_up_marker = Poly3DCollection([], facecolor="black", edgecolor="black", alpha=0.8)
    for collection in [ground_truth_sides, ground_truth_plane, estimate_sides, estimate_plane, estimate_up_marker]:
        axis.add_collection3d(collection)

    time_label = axis.text2D(0.03, 0.95, "", transform=axis.transAxes)
    axis.legend(loc="upper right")
    axis.set_title("Camera pose from IMU data, MVSEC indoor_flying")

    def update(frame_index):
        trail_start = 0
        if trail_seconds is not None:
            trail_start = max(0, frame_index - int(trail_seconds * frames_per_second))

        ground_truth_segment = ground_truth_positions[trail_start:frame_index + 1]
        estimate_segment = positions[trail_start:frame_index + 1]
        ground_truth_trail.set_data_3d(*ground_truth_segment.T)
        estimate_trail.set_data_3d(*estimate_segment.T)

        sides, plane, _ = camera_frustum_faces(ground_truth_positions[frame_index],
                                               ground_truth_camera_rotations[frame_index])
        ground_truth_sides.set_verts(sides)
        ground_truth_plane.set_verts(plane)

        sides, plane, up_marker = camera_frustum_faces(positions[frame_index], camera_rotations[frame_index])
        estimate_sides.set_verts(sides)
        estimate_plane.set_verts(plane)
        estimate_up_marker.set_verts(up_marker)

        time_label.set_text(f"t = {time[frame_index]:5.2f} s")
        return []

    animation = FuncAnimation(figure, update, frames=len(time), blit=False)
    animation.save(output_path, writer=PillowWriter(fps=frames_per_second), dpi=dpi)
    plt.close(figure)
