#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AckermannLimit(Node):
    def __init__(self):
        super().__init__('ackermann_limit')
        self.output = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_subscription(Twist, 'cmd_vel_before_steering_limit', self.limit, 1)

    def limit(self, msg):
        command = Twist()
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            # Match the base driver's speed cap and the planner's 0.40 m turning radius.
            command.linear.x = max(-0.20, min(0.20, msg.linear.x))
            max_yaw_rate = abs(command.linear.x) / 0.40
            command.angular.z = max(-max_yaw_rate, min(max_yaw_rate, msg.angular.z))
        self.output.publish(command)


def main():
    rclpy.init()
    node = AckermannLimit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
