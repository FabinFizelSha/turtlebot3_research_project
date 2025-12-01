#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand, FollowJointTrajectory
from rclpy.action import ActionClient


class CtrlNode(Node):
    def __init__(self):
        super().__init__('tb3_ctrl_node')

        # --- Arm and gripper Action clients ---
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # --- Parameters ---
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.home_position = [0.0, 0.6, -0.3, 0.0]

        self.gripper_open =  0.010
        self.gripper_closed = -0.010
        self.current_gripper_pos = 0.0
        self.gripper_step = 0.0010
        self.max_effort = 4.0

        # --- State ---
        self.state = 0  # 0=idle,1=picking,2=carrying,3=dropping

        # --- Arm label publisher ---
        self.arm_label_pub = self.create_publisher(Int8, 'arm_label', 10)
        self.create_timer(0.01, self.publish_label)  # 100 Hz

        # --- Gripper smooth control ---
        self.gripper_timer = None
        self.gripper_target = None
        self.gripper_step_direction = None
        self.gripper_callback = None

        # --- Track current arm goal ---
        self.current_arm_goal = None

        # --- IK subscriber ---
        self.create_subscription(Float32MultiArray, '/ik_joint_targets', self.target_joint_callback, 10)

        self.get_logger().info("Controller Node Initialized. Waiting for IK joint targets...")

    # ----------------------------------------------------------------------
    def publish_label(self):
        msg = Int8()
        msg.data = self.state
        self.arm_label_pub.publish(msg)

    # ----------------------------------------------------------------------
    def target_joint_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 9:
            self.get_logger().error(f"Expected 9 joint values (4 pickup + 4 drop + gripper), got {len(data)}")
            return

        pickup_joints = data[:4]
        drop_joints = data[4:8]
        gripper_target = data[8]

        # --- Cancel ongoing motions ---
        if self.gripper_timer is not None:
            self.gripper_timer.cancel()
            self.gripper_timer = None
        if self.current_arm_goal is not None:
            self.current_arm_goal.cancel_goal_async()

        # --- Start new pick/place sequence ---
        self.state = 1  # Picking
        self.send_gripper_smooth(
            self.gripper_open, self.max_effort,
            lambda: self.move_arm(
                pickup_joints,
                lambda: self.send_gripper_smooth(
                    gripper_target, self.max_effort,
                    lambda: self.move_arm_carry(drop_joints)
                )
            )
        )

    # ----------------------------------------------------------------------
    def move_arm_carry(self, drop_joints):
        self.state = 2  # Moving to drop
        self.move_arm(drop_joints, callback=self.drop_sequence)

    def drop_sequence(self):
        self.state = 3  # Dropping
        self.send_gripper_smooth(
            self.gripper_open, self.max_effort,
            self.move_arm_home
        )

    def move_arm_home(self):
        self.move_arm(self.home_position, callback=self.reset_state)

    def reset_state(self):
        self.state = 0

    # ----------------------------------------------------------------------
    def move_arm(self, positions, callback=None):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = 10
        traj.points.append(point)
        goal_msg.trajectory = traj

        self.arm_client.wait_for_server()
        self.current_arm_goal = self.arm_client.send_goal_async(goal_msg, feedback_callback=self.arm_feedback)
        self.current_arm_goal.add_done_callback(lambda f: self.arm_result_callback(f, callback))

    def arm_feedback(self, feedback_msg):
        err = feedback_msg.feedback.error.positions
        self.get_logger().info(f"[Arm Feedback] Errors: {[round(e, 4) for e in err]}")

    def arm_result_callback(self, future, callback):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._arm_done(f, callback))
        self.current_arm_goal = None  # Clear after goal finishes

    def _arm_done(self, future, callback):
        self.get_logger().info("[Arm] Motion complete")
        if callback:
            callback()

    # ----------------------------------------------------------------------
    def send_gripper_smooth(self, target_pos, max_effort, callback=None):
        if self.gripper_timer is not None:
            self.gripper_timer.cancel()
            self.gripper_timer = None

        self.gripper_target = target_pos
        self.gripper_step_direction = self.gripper_step if target_pos > self.current_gripper_pos else -self.gripper_step
        self.gripper_callback = callback
        self.gripper_timer = self.create_timer(0.05, self._gripper_step)

    def _gripper_step(self):
        if (self.gripper_step_direction > 0 and self.current_gripper_pos >= self.gripper_target) or \
           (self.gripper_step_direction < 0 and self.current_gripper_pos <= self.gripper_target):
            self.current_gripper_pos = self.gripper_target
            if self.gripper_timer is not None:
                self.gripper_timer.cancel()
                self.gripper_timer = None
            if self.gripper_callback:
                cb = self.gripper_callback
                self.gripper_callback = None
                cb()
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = self.current_gripper_pos
        goal_msg.command.max_effort = self.max_effort
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: None)
        self.current_gripper_pos += self.gripper_step_direction


def main(args=None):
    rclpy.init(args=args)
    node = CtrlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
