import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
from moveit_commander import RobotCommander, MoveGroupCommander
import rclpy
# Replace this with your actual IK solver
def calculate_ik(x, y, z):
    move_group = MoveGroupCommander("arm")  # name of your MoveIt arm group
    pose_target = move_group.get_current_pose().pose
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    # Optional: keep current orientation
    move_group.set_pose_target(pose_target)

    plan = move_group.plan()
    if plan and plan.joint_trajectory.points:
        # return first point of trajectory as joint positions
        return plan.joint_trajectory.points[0].positions
    else:
        print("IK solution not found!")
        return None

    return [x, y, z, 0.0]  # joint1, joint2, joint3, joint4

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        # Arm and Gripper action clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # Home position (joints)
        self.home_pose = [0.0, 0.0, 0.0, 0.0]

        # Gripper settings
        self.gripper_open = 0.010
        self.gripper_closed = 0.0
        self.max_effort = 0.3
        self.gripper_step = 0.0017  # step for smooth closing/opening

    def execute_pick_place(self, pickup_xyz, place_xyz):
        # Calculate joint positions using IK
        self.pickup_pose = calculate_ik(*pickup_xyz)
        self.place_pose = calculate_ik(*place_xyz)

        # Start sequence
        self.get_logger().info("Starting pick-and-place sequence")
        self.send_gripper_goal_smooth(self.gripper_open, self.max_effort, callback=self.move_to_pickup)

    # --- Arm ---
    def send_arm_goal(self, positions, callback=None):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']  # replace with your joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3
        traj.points.append(point)
        goal_msg.trajectory = traj

        self.arm_client.wait_for_server()
        future = self.arm_client.send_goal_async(goal_msg, feedback_callback=self.arm_feedback)
        future.add_done_callback(lambda f: self.arm_result_callback(f, callback))

    def arm_feedback(self, feedback_msg):
        error = feedback_msg.feedback.error.positions
        self.get_logger().info(f"[Arm Feedback] Joint errors: {[round(e,4) for e in error]}")

    def arm_result_callback(self, future, callback):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._arm_done(f, callback))

    def _arm_done(self, future, callback):
        self.get_logger().info("[Arm] Goal reached")
        if callback:
            callback()

    # --- Gripper smooth commands ---
    def send_gripper_goal_smooth(self, target_pos, max_effort, callback=None):
        """Incrementally move gripper toward target with feedback"""
        self.current_gripper_pos = self.gripper_open if target_pos > self.gripper_closed else self.gripper_closed
        step = -self.gripper_step if target_pos < self.current_gripper_pos else self.gripper_step
        self._gripper_step(target_pos, step, max_effort, callback)

    def _gripper_step(self, target, step, max_effort, callback):
        if (step > 0 and self.current_gripper_pos >= target) or (step < 0 and self.current_gripper_pos <= target):
            self.get_logger().info(f"[Gripper] Target reached: {self.current_gripper_pos:.4f}")
            if callback:
                callback()
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = self.current_gripper_pos
        goal_msg.command.max_effort = max_effort

        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.gripper_feedback_step(f, target, step, max_effort, callback))

    def gripper_feedback_step(self, future, target, step, max_effort, callback):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._gripper_done_step(f, target, step, max_effort, callback))

    def _gripper_done_step(self, future, target, step, max_effort, callback):
        result = future.result().result
        self.get_logger().info(f"[Gripper Feedback] Pos: {result.position:.4f}, Effort: {result.effort:.4f}")
        # Stop if max_effort exceeded
        if result.effort >= max_effort:
            self.get_logger().info("[Gripper] Max effort reached, stopping")
            if callback:
                callback()
            return

        # Move next step
        self.current_gripper_pos += step
        self._gripper_step(target, step, max_effort, callback)

    # --- Pick-place stages ---
    def move_to_pickup(self):
        self.get_logger().info("Moving arm to pickup location...")
        self.send_arm_goal(self.pickup_pose, callback=self.close_gripper_at_pickup)

    def close_gripper_at_pickup(self):
        self.get_logger().info("Closing gripper to pick object...")
        self.send_gripper_goal_smooth(self.gripper_closed, self.max_effort, callback=self.move_to_drop)

    def move_to_drop(self):
        self.get_logger().info("Moving arm to drop location...")
        self.send_arm_goal(self.place_pose, callback=self.open_gripper_at_drop)

    def open_gripper_at_drop(self):
        self.get_logger().info("Opening gripper to release object...")
        self.send_gripper_goal_smooth(self.gripper_open, self.max_effort, callback=self.return_home)

    def return_home(self):
        self.get_logger().info("Returning arm to home position...")
        self.send_arm_goal(self.home_pose, callback=self.sequence_complete)

    def sequence_complete(self):
        self.get_logger().info("Pick-and-place sequence completed!")

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    # Example: specify end-effector pickup and place coordinates
    pickup_xyz = [0.2, -0.3, 0.1]  # meters
    place_xyz = [0.5, 0.0, 0.1]
    node.execute_pick_place(pickup_xyz, place_xyz)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

