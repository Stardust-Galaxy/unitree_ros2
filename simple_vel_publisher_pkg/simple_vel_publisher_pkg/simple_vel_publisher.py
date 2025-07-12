#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class SimpleVelPublisher(Node):
    def __init__(self):
        super().__init__('simple_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run simple_vel_publisher_pkg simple_vel_publisher.py vx vy vyaw")
        print("Example: ros2 run simple_vel_publisher_pkg simple_vel_publisher.py 0.5 0.0 0.1")
        return
    vx = float(sys.argv[1])
    vy = float(sys.argv[2])
    vyaw = float(sys.argv[3])
    node = SimpleVelPublisher()
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = vyaw
    node.get_logger().info(f'Publishing: vx={vx}, vy={vy}, vyaw={vyaw}')
    node.publisher_.publish(msg)
    # Allow some time for the message to be sent
    rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 