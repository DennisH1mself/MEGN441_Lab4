#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.navigation_server = ActionServer(
            self, NavigateToPose, '/navigate',
            execute_callback=self.navigate_callback,
        )

    async def navigate_callback(self, goal_handle):
        # TODO: implement navigation. Abort until this callback is implemented.
        goal_handle.abort()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigation_server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
