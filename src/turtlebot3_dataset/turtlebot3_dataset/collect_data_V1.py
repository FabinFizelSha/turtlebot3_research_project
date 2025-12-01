import os
import h5py
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from cv_bridge import CvBridge
from datetime import datetime


class DatasetCollector(Node):
    def __init__(self):                                                 #Automatically runs as soon as an instance of the object is created.
        super().__init__('dataset_collector')                           #super() is a built-in Python function that lets you call methods from the parent class. "Node" in this case.
        self.bridge = CvBridge()

        # === Directory setup ===
        base_dir = os.path.expanduser('~/turtlebot_ws/data')
        os.makedirs(base_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.file_path = os.path.join(base_dir, f'session_{timestamp}.h5')

        # === Create HDF5 file structure ===
        self.file = h5py.File(self.file_path, 'a')
        self.image_grp = self.file.create_group('camera')
        self.lidar_grp = self.file.create_group('lidar')
        self.imu_grp = self.file.create_group('imu')
        self.odom_grp = self.file.create_group('odom')
        self.vel_grp = self.file.create_group('velocity')
        self.goal_grp = self.file.create_group('goal_status')

        # === Buffers and timestamps ===
        self.image_data, self.image_timestamps = [], []
        self.lidar_data, self.lidar_timestamps = [], []
        self.imu_data, self.imu_timestamps = [], []
        self.odom_data, self.odom_timestamps = [], []
        self.linear_velocity_data, self.angular_velocity_data, self.velocity_timestamps = [], [], []
        self.goal_status_data, self.goal_status_timestamps = [], []

        # === Subscribers ===
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_goal = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_cb, 10)

        self.get_logger().info(f"Dataset collector started — writing to {self.file_path}")
        self.current_goal_status = 0  # default (e.g., NO_GOAL)

    # === Helper ===
    def get_timestamp(self, msg):
        """Convert ROS2 time to float seconds."""
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # === Callbacks ===
    def image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')   #converts ROS2 image to openCV image (numpy array - bgr8 is the default OpenCV format (8-bit, Blue-Green-Red channels))
            self.image_data.append(img)
            self.image_timestamps.append(self.get_timestamp(msg))
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    def lidar_cb(self, msg):
        """Store LiDAR data with timestamp as the first element in each row."""
        ts = self.get_timestamp(msg)
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Prepend timestamp
        lidar_row = np.insert(ranges, 0, ts)
        self.lidar_data.append(lidar_row)

    def imu_cb(self, msg):
        self.imu_data.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.imu_timestamps.append(self.get_timestamp(msg))

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        self.odom_data.append([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])
        self.linear_velocity_data.append([lin.x, lin.y, lin.z])
        self.angular_velocity_data.append([ang.x, ang.y, ang.z])
        ts = self.get_timestamp(msg)
        self.odom_timestamps.append(ts)
        self.velocity_timestamps.append(ts)

    def goal_cb(self, msg):
        """Record goal status changes with consistent timestamp."""
        if len(msg.status_list) > 0:
            latest_status = msg.status_list[-1].status
            # Use the current node time (consistent with other topics)
            ts = self.get_clock().now().nanoseconds * 1e-9
            self.current_goal_status = latest_status
            self.goal_status_data.append(latest_status)
            self.goal_status_timestamps.append(ts)

    # === Cleanup / save ===
    def destroy_node(self):
        print("Saving data to HDF5 file...")

        # --- Camera ---
        if self.image_data:
            self.image_grp.create_dataset('frames', data=np.array(self.image_data), compression='gzip', chunks=True)
            self.image_grp.create_dataset('timestamps', data=np.array(self.image_timestamps))

        # --- LiDAR ---
        if self.lidar_data:
            self.lidar_grp.create_dataset('scans', data=np.array(self.lidar_data), compression='gzip', chunks=True)
            self.lidar_grp.create_dataset('timestamps', data=np.array(self.lidar_timestamps))

        # --- IMU ---
        if self.imu_data:
            self.imu_grp.create_dataset('measurements', data=np.array(self.imu_data), compression='gzip', chunks=True)
            self.imu_grp.create_dataset('timestamps', data=np.array(self.imu_timestamps))

        # --- Odometry ---
        if self.odom_data:
            self.odom_grp.create_dataset('poses', data=np.array(self.odom_data), compression='gzip', chunks=True)
            self.odom_grp.create_dataset('timestamps', data=np.array(self.odom_timestamps))

        # --- Velocity ---
        if self.linear_velocity_data or self.angular_velocity_data:
            self.vel_grp.create_dataset('Linear Velocity', data=np.array(self.linear_velocity_data), compression='gzip', chunks=True)
            self.vel_grp.create_dataset('Angular Velocity', data=np.array(self.angular_velocity_data), compression='gzip', chunks=True)
            self.vel_grp.create_dataset('timestamps', data=np.array(self.velocity_timestamps))

        # --- Goal Status ---
        if self.goal_status_data:
            self.goal_grp.create_dataset('status', data=np.array(self.goal_status_data, dtype=np.int8))
            self.goal_grp.create_dataset('timestamps', data=np.array(self.goal_status_timestamps))
            print(f"Saved {len(self.goal_status_data)} goal status updates.")

        # --- Close file ---
        self.file.close()
        print(f"Data saved to {self.file_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)                   #initializes the ROS 2 client library for Python.Think of it like “booting up” the ROS 2 environment inside your Python process.
    new_node = DatasetCollector()           #create a new instance of class DatasetCollector and calls the __init__()
                                            #Python implicitly passes the object you are calling the method on as the first argument.(self)
    try:
        rclpy.spin(new_node)                # Keep node running to process callbacks
    except KeyboardInterrupt:               # expected exit condition ctrl+C
        print('       Ctrl+C pressed. Closing Node.')
    except Exception as e:                  # unexpected error handling
        new_node.get_logger().error(f'Unexpected error: {e}')
    finally:
        new_node.destroy_node()             # Save data and clean up
        if rclpy.ok():                      # only shutdown if context is still valid
            rclpy.shutdown()                # Shut down ROS 2


if __name__ == '__main__':
    main()
