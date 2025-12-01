#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import h5py
import os
import numpy as np
import datetime  # <-- New import for timestamping


class TB3ArmLogger(Node):
    def __init__(self):
        super().__init__('tb3_arm_logger')

        # --- MODIFIED SECTION START ---
        # Generate a timestamp string for the filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        # Base directory
        base_dir = os.path.expanduser('~/data_logged/Arm')
        os.makedirs(base_dir, exist_ok=True)

        # Path to save HDF5 file, including the timestamp
        self.hdf5_file = os.path.join(base_dir, f'tb3_arm_log_{timestamp}.hdf5')
        # --- MODIFIED SECTION END ---

        # Open HDF5 file in 'w' (write/create) mode since the filename is unique
        # We use 'w' here because the file is brand new, avoiding 'a' which is for appending to an existing file
        self.file = h5py.File(self.hdf5_file, 'w')

        # Row structure remains: [label, joint1, joint2, joint3, joint4, gripper] (6 columns)
        if 'log' not in self.file:
            self.dataset = self.file.create_dataset(
                'log', shape=(0, 6), maxshape=(None, 6),
                dtype='float64', chunks=True
            )
        # Note: Since we use 'w', the 'else' block for checking an existing dataset is not needed,
        # but we keep the structure simple for compatibility if you switch back to 'a' later.

        # Latest arm label (default 0)
        self.arm_label = 0

        # Subscribers
        self.create_subscription(Int8, '/arm_label', self.label_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.gripper_pos = 0.0
        self.get_logger().info("TB3 Arm Logger Node Initialized. Logging to " + self.hdf5_file)

    def label_callback(self, msg: Int8):
        self.arm_label = msg.data

    def joint_callback(self, msg: JointState):
        # Map joint names to their positions
        name_to_pos = dict(zip(msg.name, msg.position))

        # Define the specific joint names for the arm and the gripper you want to log
        # Based on the TurtleBot3 manipulator model, the key arm joints are joint1, joint2, joint3, joint4
        ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

        # We will log only the left gripper, as the right is a mimic joint
        GRIPPER_JOINT = 'gripper_left_joint'

        # 1. Extract Arm Joints in the correct order
        joints = []
        for name in ARM_JOINTS:
            joints.append(name_to_pos.get(name, 0.0))  # Use .get with 0.0 default for safety

        # 2. Extract Gripper Joint
        gripper = name_to_pos.get(GRIPPER_JOINT, self.gripper_pos)
        self.gripper_pos = gripper  # Update for safety if the gripper joint is sometimes missing

        # Row: [label, joint1, joint2, joint3, joint4, gripper]
        row = np.array([[self.arm_label, *joints, gripper]], dtype=np.float64)

        # Append row to HDF5
        self.dataset.resize(self.dataset.shape[0] + 1, axis=0)
        self.dataset[-1:] = row
        self.file.flush()


def main(args=None):
    rclpy.init(args=args)
    node = TB3ArmLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()