import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def move(self):
        twist = Twist()

        # Step 1: move forward with linear.x = 0.5 (1 second)
        twist.linear.x = 0.5
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Moving forward: linear.x = 0.5')
        time.sleep(1.0)

        # Stop briefly
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        time.sleep(0.1)

        # Step 2: move sideways with linear.y = 0.5 (1 second)
        twist.linear.x = 0.0
        twist.linear.y = 0.5
        self.publisher_.publish(twist)
        self.get_logger().info('Moving sideways: linear.y = 0.5')
        time.sleep(5.0)

        # Stop
        twist.linear.y = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Stopped')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    node.move()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
