#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class State(Enum):
    INIT = 1
    NAVIGATE = 2
    PICK = 3
    PLACE = 4
    DONE = 5
    FAILED = 6


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.state = State.INIT
        self.BUSY = False
        self.finished = False
        self.goal_handle = None

        self.navigation_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )
        # Map-frame [x in meters, y in meters, yaw in radians].
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])
        self.declare_parameter('block_position', [0.2, 0.0, 0.0])
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.BUSY:
            return

        if self.state is State.INIT:
            self.state = State.NAVIGATE
        elif self.state is State.NAVIGATE:
            self.send_nav_goal()
        elif self.state in (State.DONE, State.FAILED):
            self.finished = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        position = feedback.current_pose.pose.position
        self.get_logger().info(
            f'Robot at ({position.x:.2f}, {position.y:.2f}); '
            f'distance remaining: {feedback.distance_remaining:.2f} m',
            throttle_duration_sec=1.0,
        )

    def send_nav_goal(self):
        if self.BUSY:
            return
        if not self.navigation_client.server_is_ready():
            self.get_logger().info(
                'Waiting for the /navigate_to_pose action server...',
                throttle_duration_sec=5.0,
            )
            return

        goal_pos = self.get_parameter('goal_position').value
        if len(goal_pos) != 3 or not all(math.isfinite(value) for value in goal_pos):
            self.navigation_failed('goal_position must contain three finite values: [x, y, yaw].')
            return
        x, y, yaw = map(float, goal_pos)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.BUSY = True
        self.get_logger().info(f'Sending navigation goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad')
        try:
            self.goal_future = self.navigation_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            self.goal_future.add_done_callback(self.goal_response_callback)
        except Exception as error:
            self.navigation_failed(f'Could not send navigation goal: {error}')

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as error:
            self.navigation_failed(f'Could not receive goal response: {error}')
            return

        if not self.goal_handle.accepted:
            self.navigation_failed('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        try:
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self.result_callback)
        except Exception as error:
            self.navigation_failed(f'Could not request navigation result: {error}')

    def result_callback(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.navigation_failed(f'Could not receive navigation result: {error}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.BUSY = False
            self.state = State.DONE
            self.get_logger().info('Navigation goal reached')
        elif status == GoalStatus.STATUS_CANCELED:
            self.navigation_failed('Navigation goal canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.navigation_failed('Navigation goal aborted')
        else:
            self.navigation_failed(f'Navigation ended with unexpected status: {status}')

    def navigation_failed(self, message):
        self.BUSY = False
        self.state = State.FAILED
        self.get_logger().error(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.state is State.FAILED else 0


if __name__ == '__main__':
    raise SystemExit(main())
