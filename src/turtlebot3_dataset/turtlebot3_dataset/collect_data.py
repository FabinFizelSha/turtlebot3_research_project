#!/usr/bin/env python3
import os
import h5py
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from cv_bridge import CvBridge
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from nav2_msgs.action import NavigateToPose


class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')
        self.bridge = CvBridge()

        # === Directory setup ===
        base_dir = os.path.expanduser('~fabin/data_logged/Navigation')
        os.makedirs(base_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.file_path = os.path.join(base_dir, f'session_{timestamp}.h5')

        # === Create HDF5 file structure ===
        self.file = h5py.File(self.file_path, 'w')
        self.image_grp = self.file.create_group('camera')
        self.lidar_grp = self.file.create_group('lidar')
        self.imu_grp = self.file.create_group('imu')
        self.odom_grp = self.file.create_group('odom')
        self.vel_grp = self.file.create_group('velocity')
        self.goal_grp = self.file.create_group('goal_status')
        self.main_status_grp = self.file.create_group('main_status')
        self.distance_grp = self.file.create_group('distance_to_goal')

        # === Buffers ===
        self.image_data = []
        self.lidar_data = []
        self.imu_data = []
        self.odom_data = []
        self.vel_data = []
        self.goal_status_data = []
        self.main_status_data = []
        self.distance_data = []

        self.latest_goal = None
        self.latest_distance = 0
        self.current_goal_status = 0
        self.last_timestamp = 0.0

        # === Subscribers ===
        self.sub_image = self.create_subscription(Image, '/pi_camera/image_raw', self.image_cb, 10)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu_broadcaster/imu', self.imu_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_goal = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_cb, 10)
        self.sub_goal_pose = self.create_subscription(PoseStamped,'/Nav_goal_position',self.goal_pose_cb,10)
        self.sub_main_status = self.create_subscription(Int8, 'main_status', self.status_cb, 10)

        self.get_logger().info(f" Dataset collector started — writing to {self.file_path}")

    # === Helpers ===
    def get_timestamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # === Callbacks ===
    def goal_pose_cb(self, msg):
        """Triggered when a new navigation goal is published."""
        self.latest_goal = msg.pose
        self.get_logger().info(
            f" New goal received: x={self.latest_goal.position.x:.2f}, "
            f"y={self.latest_goal.position.y:.2f}"
        )

    def image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = self.get_timestamp(msg)
            self.image_data.append((ts, img))
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    def lidar_cb(self, msg):
        ts = self.get_timestamp(msg)
        ranges = np.array(msg.ranges, dtype=np.float32)
        self.lidar_data.append(np.insert(ranges, 0, ts))
        self.last_timestamp = ts

    def imu_cb(self, msg):
        ts = self.get_timestamp(msg)
        imu_row = np.array([
            ts,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)
        self.imu_data.append(imu_row)

    def odom_cb(self, msg):
        ts = self.get_timestamp(msg)
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        # Save odometry and velocity
        self.odom_data.append(np.array([ts, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w], dtype=np.float32))
        self.vel_data.append(np.array([ts, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z], dtype=np.float32))

        # --- Compute distance to goal ---
        if self.latest_goal is not None:
            gx = self.latest_goal.position.x
            gy = self.latest_goal.position.y
            dx = pos.x - gx
            dy = pos.y - gy
            distance = math.sqrt(dx**2 + dy**2)
            self.latest_distance = distance
            self.distance_data.append(np.array([ts, distance], dtype=np.float32))

    def goal_cb(self, msg):
        if len(msg.status_list) > 0:
            latest_status = msg.status_list[-1].status
            ts = float(self.last_timestamp)
            self.goal_status_data.append(np.array([ts, latest_status], dtype=np.float32))
            self.current_goal_status = latest_status


    def status_cb(self, msg):
        ts = float(self.last_timestamp)
        if hasattr(self, 'latest_distance') and self.latest_distance < 0.3 and msg.data != 0:
            self.main_status_data.append(np.array([ts, 3 ], dtype=np.float32)) # main status =3 : Parking
        else:
            self.main_status_data.append(np.array([ts, msg.data], dtype=np.float32))


    # === Cleanup / save ===
    def destroy_node(self):
        print("\n Saving data to HDF5 file...")

        # --- Camera ---
        if self.image_data:
            timestamps = np.array([t for t, _ in self.image_data], dtype=np.float32)
            images = np.stack([img for _, img in self.image_data], axis=0)
            self.image_grp.create_dataset('frames', data=images, compression='gzip')
            self.image_grp.create_dataset('timestamps', data=timestamps)

        # --- LiDAR ---
        if self.lidar_data:
            self.lidar_grp.create_dataset('scans', data=np.array(self.lidar_data, dtype=np.float32), compression='gzip')

        # --- IMU ---
        if self.imu_data:
            self.imu_grp.create_dataset('measurements', data=np.array(self.imu_data, dtype=np.float32), compression='gzip')

        # --- Odometry ---
        if self.odom_data:
            self.odom_grp.create_dataset('poses', data=np.array(self.odom_data, dtype=np.float32), compression='gzip')

        # --- Velocity ---
        if self.vel_data:
            self.vel_grp.create_dataset('velocity', data=np.array(self.vel_data, dtype=np.float32), compression='gzip')

        # --- Goal Status ---
        if self.goal_status_data:
            self.goal_grp.create_dataset('status', data=np.array(self.goal_status_data, dtype=np.int32), compression='gzip')
            print(f"✅ Saved {len(self.goal_status_data)} goal status updates.")

        # --- Main Status ---
        if self.main_status_data:
            self.main_status_grp.create_dataset('status', data=np.array(self.main_status_data, dtype=np.int32), compression='gzip')
            print(f"✅ Saved {len(self.main_status_data)} main status entries.")

        # --- Distance to Goal ---
        if self.distance_data:
            self.distance_grp.create_dataset('values', data=np.array(self.distance_data, dtype=np.float32), compression='gzip')
            print(f"✅ Saved {len(self.distance_data)} distance entries.")
        else:
            print("⚠ No distance_to_goal data recorded (no goals were received).")

        self.file.close()
        print(f" Data saved to {self.file_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n Ctrl+C pressed. Closing node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
