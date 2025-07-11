import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        self.publisher = self.create_publisher(Point, '/end_effector_position', 10)

    def joint_callback(self, msg):
        try:
            theta1 = msg.position[0] + math.pi / 2  # Adjusted as per problem
            theta2 = msg.position[1]
            L1 = 2.0
            L2 = 1.5

            x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
            y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = 0.0  # Not used

            self.publisher.publish(point_msg)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
