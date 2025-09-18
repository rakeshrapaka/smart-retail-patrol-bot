import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(5.0, self.patrol)

    def patrol(self):
        msg = Twist()
        msg.linear.x = 0.2
        self.get_logger().info("Patrolling forward...")
        self.cmd_pub.publish(msg)
        time.sleep(2)
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.get_logger().info("Turning...")
        self.cmd_pub.publish(msg)
        time.sleep(2)
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
