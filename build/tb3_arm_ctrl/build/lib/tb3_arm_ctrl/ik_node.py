#!/usr/bin/env python3
"""
TurtleBot3 Arm IK Node

- Receives pickup/drop coordinates (Float32MultiArray)
- Computes IK using MoveIt
- Publishes IK joint targets to /ik_joint_targets
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

import threading


class TB3IKNode(Node):
    def __init__(self):
        super().__init__('tb3_ik_node')
        self.get_logger().info("Initializing TB3 IK Node...")

        self.callback_group = ReentrantCallbackGroup()

        # ---- Arm configuration ----
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.planning_frame = "base_link"
        self.end_effector_link = "end_effector_link"
        self.arm_group_name = "arm"
        self.planning_time = 5.0

        # ---- Joint state storage ----
        self.current_joint_states = None
        self.joint_states_lock = threading.Lock()

        # ---- Subscribers ----
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.create_subscription(
            Float32MultiArray, '/arm_goal_xyz', self.goal_callback, 10
        )

        # ---- Publisher (IK solutions) ----
        self.ik_solution_pub = self.create_publisher(
            Float32MultiArray, '/ik_joint_targets', 10
        )
        # ---- temp storage ---

        self.pending_pickup = None
        self.pending_drop = None


        # ---- IK Service client ----
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.callback_group
        )

        self.get_logger().info("Waiting for IK service...")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("IK service not available!")
        else:
            self.get_logger().info("IK service connected.")

    # ----------------------------------------------------
    # Joint State Callback
    # ----------------------------------------------------
    def joint_state_callback(self, msg):
        with self.joint_states_lock:
            self.current_joint_states = msg

    # ----------------------------------------------------
    # Goal Callback (pickup + drop)
    # ----------------------------------------------------
    def goal_callback(self, msg: Float32MultiArray):
        coords = list(msg.data)

        if len(coords) != 7:
            self.get_logger().error("Expected 7 floats (pickup[3], drop[3], pick_grip, unused).")
            return

        # Extract coordinates
        pickup_xyz = coords[:3]
        drop_xyz = coords[3:6]

        # NEW: extract pickup gripper target
        self.pick_grip = coords[6]  # ✔ correct index
        self.get_logger().info(f"Received pickup={pickup_xyz}, drop={drop_xyz}, pick_grip={self.pick_grip}")

        # Trigger IK
        self.compute_ik_async(*pickup_xyz, which='pickup')
        self.compute_ik_async(*drop_xyz, which='drop')

    # ----------------------------------------------------
    # Get joint states
    # ----------------------------------------------------
    def get_current_joint_values(self):
        with self.joint_states_lock:
            if self.current_joint_states is None:
                return None

            try:
                return [
                    self.current_joint_states.position[
                        self.current_joint_states.name.index(j)
                    ]
                    for j in self.arm_joint_names
                ]
            except ValueError:
                self.get_logger().error("JointState missing required joints.")
                return None

    # ----------------------------------------------------
    # IK Request
    # ----------------------------------------------------
    def compute_ik_async(self, x, y, z, orientation=None, which='goal'):
        self.get_logger().info(f"Computing IK for {which}: ({x}, {y}, {z})")

        current_joints = self.get_current_joint_values()
        if current_joints is None:
            self.get_logger().warn("Joint states not available yet.")
            return

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # SAFE ORIENTATION
        pose.orientation = orientation or Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0
        )

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.arm_group_name
        req.ik_request.robot_state.joint_state.name = self.arm_joint_names
        req.ik_request.robot_state.joint_state.position = current_joints

        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = self.planning_frame
        stamped_pose.header.stamp = self.get_clock().now().to_msg()
        stamped_pose.pose = pose

        req.ik_request.pose_stamped = stamped_pose
        req.ik_request.ik_link_name = self.end_effector_link
        req.ik_request.timeout.sec = int(self.planning_time)

        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda fut, w=which: self.ik_response_callback(fut, w))

    # ----------------------------------------------------
    # IK Response
    # ----------------------------------------------------
    def ik_response_callback(self, future, which='goal'):
        try:
            resp = future.result()

            if resp.error_code.val != resp.error_code.SUCCESS:
                self.get_logger().error(
                    f"{which.upper()} IK FAILED, error code {resp.error_code.val}"
                )
                return

            # Extract joint positions in correct order
            joint_positions = [
                resp.solution.joint_state.position[
                    resp.solution.joint_state.name.index(j)
                ]
                for j in self.arm_joint_names
            ]

            # Store depending on which solution it is
            if which == 'pickup':
                self.pending_pickup = joint_positions
                self.get_logger().info(f"Stored PICKUP IK: {joint_positions}")

            elif which == 'drop':
                self.pending_drop = joint_positions
                self.get_logger().info(f"Stored DROP IK: {joint_positions}")

            # Check if both IK results are ready
            if self.pending_pickup is not None and self.pending_drop is not None:
                combined = self.pending_pickup + self.pending_drop
                combined.append(self.pick_grip)

                msg = Float32MultiArray()
                msg.data = combined
                self.ik_solution_pub.publish(msg)

                self.get_logger().info(f"Published COMBINED IK + Grip: {combined}")
                # Reset for next goal
                self.pending_pickup = None
                self.pending_drop = None

        except Exception as e:
            self.get_logger().error(f"IK callback error: {e}")


# ----------------------------------------------------
# Main
# ----------------------------------------------------
def main():
    rclpy.init()
    node = TB3IKNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
