import os
import h5py
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp
from datetime import datetime

# --- Input HDF5 file ---
input_file = '/home/fabin/data_logged/Navigation/session_2025-12-08_23-18-24.h5'

# --- Open raw dataset ---
f = h5py.File(input_file, 'r')

# === Reference: LiDAR timestamps ===
lidar_scans = np.array(f['lidar/scans'])
lidar_time = lidar_scans[:, 0]  # first column = timestamps
lidar_ranges = lidar_scans[:, 1:]  # remaining columns = scan values



# --- Helper: interpolate numeric sensors ---
def interpolate_to_reference(data_arr, ref_time):
    """Interpolate numeric columns to reference time (clamped)."""
    data_time = data_arr[:, 0]
    n_cols = data_arr.shape[1] - 1
    aligned_vals = np.zeros((len(ref_time), n_cols), dtype=np.float32)

    for i in range(n_cols):
        interp_func = interp1d(
            data_time,
            data_arr[:, i + 1],
            kind='linear',
            bounds_error=False,
            fill_value=(data_arr[0, i + 1], data_arr[-1, i + 1])
        )
        aligned_vals[:, i] = interp_func(ref_time)
    return np.column_stack((ref_time, aligned_vals))


# --- Helper: quaternion SLERP with clamping ---
def interpolate_quaternions(data_time, quaternions, ref_time):
    """Spherical linear interpolation (SLERP) of quaternions with clamped reference time."""
    # Clamp reference time to valid range
    ref_time_clamped = np.clip(ref_time, data_time[0], data_time[-1])

    # Create Rotation object
    rotations = Rotation.from_quat(quaternions)

    # Slerp object
    slerp = Slerp(data_time, rotations)

    # Evaluate
    interp_rots = slerp(ref_time_clamped)
    return interp_rots.as_quat()


# --- Load odometry and interpolate ---
odom_arr = np.array(f['odom/poses'])
odom_time = odom_arr[:, 0]
pos_arr = odom_arr[:, 1:4]
quat_arr = odom_arr[:, 4:8]

# Linear interpolate positions
pos_interp = np.stack([
    interp1d(odom_time, pos_arr[:, i], kind='linear', bounds_error=False,
             fill_value=(pos_arr[0, i], pos_arr[-1, i]))(lidar_time)
    for i in range(3)
], axis=1)

# SLERP quaternions
quat_interp = interpolate_quaternions(odom_time, quat_arr, lidar_time)

# Combine into aligned odometry
odom_aligned = np.column_stack((lidar_time, pos_interp, quat_interp))

# --- Load & interpolate velocity and IMU ---
vel_arr = np.array(f['velocity/velocity'])
vel_aligned = interpolate_to_reference(vel_arr, lidar_time)

imu_arr = np.array(f['imu/measurements'])
imu_aligned = interpolate_to_reference(imu_arr, lidar_time)

# --- Camera alignment (nearest neighbor) ---
cam_frames = np.array(f['camera/frames'])
cam_time = np.array(f['camera/timestamps'])


def nearest_neighbor_indices(sensor_time, ref_time):
    idx = np.searchsorted(sensor_time, ref_time)
    idx0 = np.clip(idx - 1, 0, len(sensor_time) - 1)
    idx1 = np.clip(idx, 0, len(sensor_time) - 1)
    return np.where(np.abs(sensor_time[idx0] - ref_time) <= np.abs(sensor_time[idx1] - ref_time), idx0, idx1)


cam_indices = nearest_neighbor_indices(cam_time, lidar_time)
cam_aligned = cam_frames[cam_indices]

# --- Reassemble LiDAR dataset with timestamps ---
lidar_aligned = np.column_stack((lidar_time, lidar_ranges))

# --- Output file ---
timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_file = os.path.expanduser(f'/home/fabin/data_logged/Navigation synchronized/{timestamp_str}.h5')
os.makedirs(os.path.dirname(output_file), exist_ok=True)

# --- Write synchronized HDF5 ---
with h5py.File(output_file, 'w') as hf:
    # LiDAR
    lidar_grp = hf.create_group('lidar')
    lidar_grp.create_dataset('scans', data=lidar_aligned, compression='gzip')

    # Camera
    cam_grp = hf.create_group('camera')
    cam_grp.create_dataset('frames', data=cam_aligned, compression='gzip')
    cam_grp.create_dataset('timestamps', data=lidar_time)
    cam_grp.create_dataset('original_timestamps', data=cam_time)
    cam_grp.create_dataset('lidar_to_cam_index', data=cam_indices)

    # Odometry
    odom_grp = hf.create_group('odom')
    odom_grp.create_dataset('poses', data=odom_aligned, compression='gzip')

    # Velocity
    vel_grp = hf.create_group('velocity')
    vel_grp.create_dataset('velocity', data=vel_aligned, compression='gzip')

    # IMU
    imu_grp = hf.create_group('imu')
    imu_grp.create_dataset('measurements', data=imu_aligned, compression='gzip')

    # Copy unaltered groups
    if 'goal_status' in f:
        f.copy('goal_status', hf)
    if 'main_status' in f:
        f.copy('main_status', hf)

print(f"✅ Synchronized dataset saved to {output_file}")
