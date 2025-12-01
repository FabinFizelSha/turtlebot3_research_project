import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node')

        # Publisher to send XYZ + gripper closing goal
        self.publisher_ = self.create_publisher(Float32MultiArray, 'arm_goal_xyz', 10)

        # Publish once after startup
        self.timer = self.create_timer(1.0, self.publish_goal)

        # Generate random coordinate within safe workspace
        def random_xyz():
            x = random.uniform(0.10, 0.25)
            y = random.uniform(-0.25, 0.25)
            z = random.uniform(0.10, 0.20)
            return [x, y, z]

        # Predefined pickup and drop coordinates (now random)
        self.pickup_xyz = random_xyz()
        self.drop_xyz = random_xyz()
        # Generate gripper closing goal between -0.1 and +0.1
        self.gripper_goal_pos = random.uniform(-0.01, 0.01)

        self.published = False

    def publish_goal(self):
        if self.published:
            return

        msg = Float32MultiArray()

        # Message format:
        # [pick_x, pick_y, pick_z,
        #  drop_x, drop_y, drop_z,
        #  gripper_goal_position]
        msg.data = (
            self.pickup_xyz +
            self.drop_xyz +
            [self.gripper_goal_pos]
        )

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published XYZ + gripper goal: {msg.data}")

        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = GoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
