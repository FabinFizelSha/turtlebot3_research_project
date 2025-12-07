#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import random
import math
import numpy as np

# Fix for transforms3d / tf_transformations with new NumPy
np.float = float
from tf_transformations import quaternion_from_euler


class RandomNavGoalSender(Node):
    def __init__(self, n_goals=10, wait_time=2.0):
        super().__init__('random_nav_goal_sender')

        self.n_goals = n_goals
        self.wait_time = wait_time

        # === Action client for navigation ===
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # === Publisher for Main Status ===
        self.main_status_pub = self.create_publisher(Int8, 'main_status', 10)
        self.IDLE = 0
        self.ACTIVE = 1
        self.current_status = self.IDLE

        # === Publisher for selected goal pose ===
        self.goal_pub = self.create_publisher(PoseStamped, 'Nav_goal_position', 10)

        # === Timer for continuous 5 Hz publishing of main_status ===
        self.timer_period = 0.2  # 5 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_status_timer)

        # === Map boundaries (adjust to your environment) ===
        self.x_min, self.x_max = -2.0, 2.0
        self.y_min, self.y_max = -2.0, 2.0

        self.get_logger().info('Random Navigation Goal Sender Node Started')

    # --- Timer callback ---
    def publish_status_timer(self):
        msg = Int8()
        msg.data = self.current_status
        self.main_status_pub.publish(msg)

    # --- Send random goals sequentially ---
    def send_random_goals(self):
        for i in range(self.n_goals):
            # Switch to ACTIVE
            self.current_status = self.ACTIVE

            goal_msg = NavigateToPose.Goal()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            # Random position
            pose.pose.position.x = random.uniform(self.x_min, self.x_max)
            pose.pose.position.y = random.uniform(self.y_min, self.y_max)
            pose.pose.position.z = 0.0

            # Random yaw
            yaw = random.uniform(-math.pi, math.pi)
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            goal_msg.pose = pose

            # --- Publish the selected goal pose ---
            self.goal_pub.publish(pose)

            self.get_logger().info(
                f'Sending goal {i + 1}/{self.n_goals}: '
                f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, yaw={yaw:.2f}'
            )

            # --- Send goal asynchronously ---
            future = self.nav_to_pose_client.send_goal_async(goal_msg)
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected')
                continue

            # --- Wait for result asynchronously ---
            result_future = goal_handle.get_result_async()
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info(f'Goal {i + 1} reached.')

            # Switch to IDLE **before waiting**
            self.current_status = self.IDLE

            # Wait period while still spinning
            t_end = self.get_clock().now().nanoseconds * 1e-9 + self.wait_time
            while self.get_clock().now().nanoseconds * 1e-9 < t_end:
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('All goals completed. Node finished.')

    def destroy_node(self):
        self.get_logger().info('Shutting down RandomNavGoalSender...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomNavGoalSender(n_goals=10, wait_time=2.0)

    # Wait for Nav2 action server
    if not node.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('NavigateToPose action server not available!')
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        node.send_random_goals()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

