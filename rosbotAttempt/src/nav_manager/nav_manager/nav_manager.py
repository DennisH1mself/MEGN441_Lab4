"""Nav2 action relay, written in the same callback style as the Waypoint lab.

The delivery sequencer is the mission state machine.  This node is the
navigation *server* for /navigate and an action *client* of Nav2.
"""
import threading
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.busy = False
        self.lock = threading.Lock()
        self.group = ReentrantCallbackGroup()

        self.client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.group)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate',
            execute_callback=self.action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group)
        self.get_logger().info('Navigation relay ready at /navigate')

    def goal_callback(self, request):
        with self.lock:
            if self.busy or not self.client.server_is_ready():
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Navigation cancellation requested')
        return CancelResponse.ACCEPT

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Nav2 remaining: {fb.distance_remaining:.2f} m',
            throttle_duration_sec=2.0)

    def goal_response_callback(self, future):
        # Nav2 has responded: was our forwarded goal accepted?
        try:
            self.nav_goal_handle = future.result()
            if not self.nav_goal_handle.accepted:
                self.nav_error = 'Nav2 rejected navigation goal'
                return
            self.nav_result_future = self.nav_goal_handle.get_result_async()
            self.nav_result_future.add_done_callback(self.result_callback)
        except Exception as error:
            self.nav_error = f'Nav2 goal response failed: {error}'

    def result_callback(self, future):
        # Final Nav2 result, including succeeded / canceled / aborted.
        try:
            self.nav_result = future.result()
        except Exception as error:
            self.nav_error = f'Nav2 result failed: {error}'

    async def wait_tick(self):
        future = Future(executor=self.executor)
        def wake():
            if not future.done():
                future.set_result(None)
        timer = self.create_timer(0.1, wake, callback_group=self.group)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def action_callback(self, goal_handle):
        # Accept sequencer goal, forward it to Nav2, and await completion.
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_result = None
        self.nav_error = None
        cancel_sent = False
        try:
            send_future = self.client.send_goal_async(
                goal_handle.request,
                feedback_callback=self.feedback_callback)
            send_future.add_done_callback(self.goal_response_callback)

            while self.nav_result is None and self.nav_error is None:
                if (goal_handle.is_cancel_requested and
                        self.nav_goal_handle is not None and
                        not cancel_sent):
                    self.nav_goal_handle.cancel_goal_async()
                    cancel_sent = True
                await self.wait_tick()

            if self.nav_error:
                self.get_logger().error(self.nav_error)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return NavigateToPose.Result()

            response = self.nav_result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif response.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return response.result
        except Exception as error:
            self.get_logger().error(f'Navigation relay failed: {error}')
            if self.nav_goal_handle is not None and self.nav_goal_handle.accepted:
                self.nav_goal_handle.cancel_goal_async()
            if goal_handle.is_active:
                goal_handle.abort()
            return NavigateToPose.Result()
        finally:
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
