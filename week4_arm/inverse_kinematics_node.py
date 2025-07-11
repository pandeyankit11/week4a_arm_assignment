import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.subscription = self.create_subscription(Point, '/end_effector_position', self.position_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_angles_goal', 10)
        self.current_x = None
        self.current_y = None

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.get_logger().info(f"Current Position: x={self.current_x}, y={self.current_y}")
        self.move_prompt()

    def move_prompt(self):
        direction = input("Enter direction (x/y): ").strip()
        dist = float(input("Enter distance (<= 0.5 m): "))
        if abs(dist) > 0.5:
            print("Too large! Max allowed is 0.5m")
            return

        new_x = self.current_x
        new_y = self.current_y

        if direction == 'x':
            new_x += dist
        elif direction == 'y':
            new_y += dist
        else:
            print("Invalid direction!")
            return

        if not self.is_reachable(new_x, new_y):
            self.get_logger().warn("Target position is unreachable.")
            return

        theta1, theta2 = self.compute_ik(new_x, new_y)
        msg = Float64MultiArray()
        msg.data = [theta1, theta2]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: θ1={theta1:.2f}, θ2={theta2:.2f}")

    def is_reachable(self, x, y):
        L1 = 2.0
        L2 = 1.5
        D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        return abs(D) <= 1.0

    def compute_ik(self, x, y):
        L1 = 2.0
        L2 = 1.5
        D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = math.acos(D)
        theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
        return theta1, theta2

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
