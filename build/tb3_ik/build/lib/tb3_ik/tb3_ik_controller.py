#!/usr/bin/env python3

"""
TurtleBot3 Arm IK Node with Manual Gripper Control (Gazebo)

- Computes IK for pick/place positions.
- Publishes joint trajectories to the arm controller.
- Gripper must be controlled manually via action or separate node.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand

import threading


class TurtleBot3ArmIK(Node):
    def __init__(self):
        super().__init__('tb3_arm_ik_controller')
        self.callback_group = ReentrantCallbackGroup()

        # --- Arm configuration ---
        self.arm_group_name = "arm"
        self.planning_frame = "base_link"
        self.end_effector_link = "end_effector_link"
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.planning_time = 5.0

        self.current_joint_states = None
        self.joint_states_lock = threading.Lock()

        # --- IK Service Client ---
        self.ik_service = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.callback_group
        )

        # --- Arm Trajectory Publisher ---
        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # --- Gripper Action Client (manual use) ---
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_cmd')

        # --- Subscribers ---
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        self.pick_position_sub = self.create_subscription(
            Point,
            '/pick_position',
            self.pick_position_callback,
            10,
            callback_group=self.callback_group
        )
        self.place_position_sub = self.create_subscription(
            Point,
            '/place_position',
            self.place_position_callback,
            10,
            callback_group=self.callback_group
        )
        self.place_object_sub = self.create_subscription(
            String,
            '/place_object',
            self.place_object_callback,
            10,
            callback_group=self.callback_group
        )

        # --- Optional target color publisher ---
        self.target_color_pub = self.create_publisher(String, '/target_color', 10)

        self.get_logger().info("TurtleBot3 OpenManipulator IK Controller Initialized (Manual Gripper Control)")
        self.get_logger().info(f"Planning frame: {self.planning_frame}")
        self.get_logger().info(f"End effector link: {self.end_effector_link}")

        self.wait_for_services()

    # -----------------------------
    # Utility Functions
    # -----------------------------
    def wait_for_services(self):
        self.get_logger().info("Waiting for IK service...")
        if not self.ik_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("IK service not available. MoveIt must be running.")
        else:
            self.get_logger().info("IK service connected.")

    def joint_state_callback(self, msg):
        with self.joint_states_lock:
            self.current_joint_states = msg

    def get_current_joint_values(self):
        with self.joint_states_lock:
            if self.current_joint_states is None:
                return None
            joint_values = []
            for joint_name in self.arm_joint_names:
                try:
                    idx = self.current_joint_states.name.index(joint_name)
                    joint_values.append(self.current_joint_states.position[idx])
                except ValueError:
                    self.get_logger().warn(f"Joint {joint_name} not found in joint states")
                    return None
            return joint_values

    def publish_target_color(self, color='blue'):
        msg = String()
        msg.data = color
        self.target_color_pub.publish(msg)
        self.get_logger().info(f"Published target color: {color}")

    # -----------------------------
    # Pick/Place Callbacks
    # -----------------------------
    def pick_position_callback(self, msg):
        self.get_logger().info(f"Received pick position: x={msg.x}, y={msg.y}, z={msg.z}")
        self.compute_ik_async(msg.x, msg.y, msg.z, which='pick')

    def place_position_callback(self, msg):
        self.get_logger().info(f"Received place position: x={msg.x}, y={msg.y}, z={msg.z}")
        self.compute_ik_async(msg.x, msg.y, msg.z, which='place')

    def place_object_callback(self, msg):
        self.get_logger().info("Received place object command")
        self.compute_ik_async(0.28, 0.0106, 0.199, which='place')

    # -----------------------------
    # IK Computation
    # -----------------------------
    def compute_ik_async(self, x, y, z, orientation=None, which='pick'):
        pose_target = Pose()
        pose_target.position = Point(x=x, y=y, z=z)
        pose_target.orientation = orientation or Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)

        current_joint_values = self.get_current_joint_values()
        if current_joint_values is None:
            self.get_logger().warn("Current joint values not available yet.")
            return

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.arm_group_name
        ik_request.ik_request.robot_state.joint_state.header.frame_id = self.planning_frame
        ik_request.ik_request.robot_state.joint_state.name = self.arm_joint_names
        ik_request.ik_request.robot_state.joint_state.position = current_joint_values

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose_target
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.ik_link_name = self.end_effector_link
        ik_request.ik_request.timeout.sec = int(self.planning_time)
        ik_request.ik_request.timeout.nanosec = int((self.planning_time % 1) * 1e9)

        self.get_logger().info(f"Computing IK for {which} position...")
        future = self.ik_service.call_async(ik_request)
        future.add_done_callback(lambda fut: self.ik_result_callback(fut, which))

    def ik_result_callback(self, future, which):
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                joint_values = []
                for joint_name in self.arm_joint_names:
                    idx = response.solution.joint_state.name.index(joint_name)
                    joint_values.append(response.solution.joint_state.position[idx])

                self.publish_joint_trajectory(joint_values, which)
                self.get_logger().info(f"{which.capitalize()} joint values applied: {joint_values}")
            else:
                self.get_logger().error(f"IK failed with error code: {response.error_code.val}")
                self.publish_target_color('red')
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")

    # -----------------------------
    # Trajectory Publisher
    # -----------------------------
    def publish_joint_trajectory(self, joint_values, which='pick'):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_values
        point.time_from_start.sec = 2
        traj_msg.points.append(point)
        self.arm_trajectory_pub.publish(traj_msg)
        self.get_logger().info(f"Trajectory for {which} published to /arm_controller/joint_trajectory")


# -----------------------------
# Main
# -----------------------------
def main():
    rclpy.init()
    node = TurtleBot3ArmIK()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

