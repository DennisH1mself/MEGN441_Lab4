#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.busy = False
        self.navigation_goal = None
        self.navigation_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.navigation_server = ActionServer(
            self, NavigateToPose, '/navigate',
            execute_callback=self.navigate_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, request):
        if self.busy:
            return GoalResponse.REJECT
        if not self.navigation_client.server_is_ready():
            self.get_logger().warn('Nav2 /navigate_to_pose is not ready.')
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        if self.navigation_goal is not None:
            self.navigation_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def feedback_callback(self, goal_handle, feedback_msg):
        feedback = feedback_msg.feedback
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback)
        position = feedback.current_pose.pose.position
        self.get_logger().info(
            f'Robot at ({position.x:.2f}, {position.y:.2f}); '
            f'distance remaining: {feedback.distance_remaining:.2f} m',
            throttle_duration_sec=1.0,
        )

    async def navigate_callback(self, goal_handle):
        try:
            self.navigation_goal = await self.navigation_client.send_goal_async(
                goal_handle.request,
                feedback_callback=lambda feedback: self.feedback_callback(
                    goal_handle, feedback),
            )
            if not self.navigation_goal.accepted:
                self.get_logger().error('Nav2 rejected the navigation goal.')
                goal_handle.abort()
                return NavigateToPose.Result()

            # Cancellation can arrive while Nav2 is accepting the goal.
            if goal_handle.is_cancel_requested:
                self.navigation_goal.cancel_goal_async()
            response = await self.navigation_goal.get_result_async()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif response.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return response.result
        except Exception as error:
            self.get_logger().error(f'Navigation failed: {error}')
            goal_handle.abort()
            return NavigateToPose.Result()
        finally:
            self.navigation_goal = None
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigation_server.destroy()
        node.navigation_client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
