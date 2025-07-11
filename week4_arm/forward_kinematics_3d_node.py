import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class FK3DNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_3d_node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        self.publisher = self.create_publisher(Point, '/end_effector_position', 10)

    def joint_callback(self, msg):
        try:
            theta0 = msg.position[0]  # base yaw
            theta1 = msg.position[1]  # shoulder pitch
            theta2 = msg.position[2]  # elbow pitch

            L1 = 2.0
            L2 = 1.5

            # Compute r and z in 2D
            r = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
            z = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

            # Rotate r into X and Y using base yaw
            x = r * math.cos(theta0)
            y = r * math.sin(theta0)

            point = Point()
            point.x = x
            point.y = y
            point.z = z

            self.publisher.publish(point)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FK3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
