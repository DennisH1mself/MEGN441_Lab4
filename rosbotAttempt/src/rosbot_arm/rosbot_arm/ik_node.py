import numpy as np
import threading
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rosbot_msgs.msg import IKTarget, JointControl
from rosbot_msgs.action import Move


##  Joint Links ##
# l0 is the shoulder height above base_footprint.
l0 = 0.05 + 0.0654868 + 0.0338648 + 0.0772047
l1 = 0.130
l2 = 0.130
l3 = 0.055
tool_link = 0.117

# Servo-angle limits for IDs 1 through 5; shoulder protects the screen.
min_angles = [-120, -90, -120, -120, -120]
max_angles = [120, 66, 120, 120, 120]
STOW_POSE = dict(x=0.18, y=0.02, z=0.32, roll=0.0, pitch=-30.0)


class RangeError(ValueError):
    pass


def inverse_kinematics(target: IKTarget):
    x = target.x
    y = target.y
    z = target.z
    roll = target.roll
    pitch = target.pitch
    if not np.all(np.isfinite([x, y, z, roll, pitch])):
        raise ValueError('IK target must contain finite numbers.')
    radius = np.hypot(x, y)
    if radius < 1e-9:
        raise RangeError('Target is on the base axis; rotation is undefined.')
    rotation = np.rad2deg(np.arctan2(y, x))
    # Wrist center relative to the shoulder; preserve the radial sign.
    v3z = z - l0 - (l3 + tool_link) * np.sin(np.deg2rad(pitch))
    v3rxy = radius - (l3 + tool_link) * np.cos(np.deg2rad(pitch))
    distance = np.hypot(v3rxy, v3z)
    if distance < 1e-9:
        raise RangeError('Wrist center is at the shoulder; IK is singular.')
    if distance > l1 + l2 + 1e-9 or distance < abs(l1 - l2) - 1e-9:
        raise RangeError('Target is outside the arm reach.')

    elbow = np.arccos(np.clip((distance**2 - l1**2 - l2**2) / (2 * l1 * l2), -1.0, 1.0))
    # Try both elbow bends; servo shoulder zero is upright (90 degrees).
    for theta_2 in (elbow, -elbow):
        theta_1 = np.arctan2(v3z, v3rxy) - np.arctan2(l2 * np.sin(theta_2), l1 + l2 * np.cos(theta_2))
        theta_3 = pitch - np.rad2deg(theta_1 + theta_2)
        angles = np.array([rotation, np.rad2deg(theta_1) - 90.0, np.rad2deg(theta_2), theta_3, roll])
        angles = (angles + 180.0) % 360.0 - 180.0
        if np.all((angles >= np.array(min_angles) - 1e-9) & (angles <= np.array(max_angles) + 1e-9)):
            return angles
    raise ValueError('Neither elbow bend fits the calibrated servo joint limits.')


class ActionCanceled(Exception):
    pass


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.publisher = self.create_publisher(
            JointControl, '/joint_control', 10)
        self.open_grip = 100
        self.close_grip = 500
        self.last_angles = None
        self.cur_grip = self.open_grip
        self.stow_position = IKTarget(**STOW_POSE)
        self.busy = False
        self._lock = threading.Lock()
        self.action_group = ReentrantCallbackGroup()

        self.stow_server = ActionServer(
            self, Move, '/stow', execute_callback=self.stow_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback, callback_group=self.action_group)
        self.pick_server = ActionServer(
            self, Move, '/pick', execute_callback=self.pick_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback, callback_group=self.action_group)
        self.place_server = ActionServer(
            self, Move, '/place', execute_callback=self.place_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback, callback_group=self.action_group)

    def goal_callback(self, request):
        with self._lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def stow(self):
        self.write_joints(inverse_kinematics(self.stow_position), self.cur_grip)

    async def pause(self, goal_handle):
        # Two-second wait using ROS, rather than an asyncio event loop.
        future = Future(executor=self.executor)

        def wake():
            timer.cancel()
            future.set_result(None)

        timer = self.create_timer(2.0, wake, callback_group=self.action_group)
        try:
            await future
        finally:
            self.destroy_timer(timer)
        if goal_handle.is_cancel_requested:
            raise ActionCanceled()

    async def stow_callback(self, goal_handle):
        result = Move.Result()
        try:
            if goal_handle.is_cancel_requested:
                raise ActionCanceled()
            self.stow()
            await self.pause(goal_handle)
            goal_handle.succeed()
            result.succeeded = True
        except ActionCanceled:
            goal_handle.canceled()
        except Exception as error:
            self.get_logger().error(f'Stow failed: {error}')
            goal_handle.abort()
            result.error_occured = True
        finally:
            with self._lock:
                self.busy = False
        return result

    async def pick_callback(self, goal_handle):
        result = Move.Result()
        feedback = Move.Feedback(busy=True)
        try:
            if goal_handle.is_cancel_requested:
                raise ActionCanceled()
            angles = inverse_kinematics(goal_handle.request)
            inverse_kinematics(self.stow_position)
            feedback.in_range = True

            self.write_joints(angles, self.open_grip)
            feedback.status = 'Moving to pick position'
            goal_handle.publish_feedback(feedback)
            await self.pause(goal_handle)

            self.write_joints(angles, self.close_grip)
            feedback.status = 'Closing gripper'
            goal_handle.publish_feedback(feedback)
            await self.pause(goal_handle)

            self.stow()
            feedback.status = 'Returning to stow position'
            goal_handle.publish_feedback(feedback)
            await self.pause(goal_handle)

            goal_handle.succeed()
            result.succeeded = True
            feedback.status = 'Pick completed'
        except ActionCanceled:
            goal_handle.canceled()
            feedback.status = 'Pick canceled'
        except Exception as error:
            goal_handle.abort()
            result.error_occured = True
            feedback.status = str(error)
        finally:
            with self._lock:
                self.busy = False
            feedback.busy = False
            goal_handle.publish_feedback(feedback)
        return result

    async def place_callback(self, goal_handle):
        result = Move.Result()
        feedback = Move.Feedback(busy=True)
        try:
            if goal_handle.is_cancel_requested:
                raise ActionCanceled()
            angles = inverse_kinematics(goal_handle.request)
            inverse_kinematics(self.stow_position)
            feedback.in_range = True

            self.write_joints(angles, self.close_grip)
            feedback.status = 'Moving to place position'
            goal_handle.publish_feedback(feedback)
            await self.pause(goal_handle)

            self.write_joints(angles, self.open_grip)
            feedback.status = 'Opening gripper'
            goal_handle.publish_feedback(feedback)
            await self.pause(goal_handle)

            self.stow()
            feedback.status = 'Returning to stow position'
            goal_handle.publish_feedback(feedback)
            await self.pause(goal_handle)

            goal_handle.succeed()
            result.succeeded = True
            feedback.status = 'Place completed'
        except ActionCanceled:
            goal_handle.canceled()
            feedback.status = 'Place canceled'
        except Exception as error:
            goal_handle.abort()
            result.error_occured = True
            feedback.status = str(error)
        finally:
            with self._lock:
                self.busy = False
            feedback.busy = False
            goal_handle.publish_feedback(feedback)
        return result

    def write_joints(self, angles, gripper):
        msg = JointControl()
        msg.rotation = int(round(angles[0]))
        msg.joint1 = int(round(angles[1]))
        msg.joint2 = int(round(angles[2]))
        msg.joint3 = int(round(angles[3]))
        msg.wrist = int(round(angles[4]))
        msg.gripper = int(gripper)
        self.publisher.publish(msg)
        self.last_angles = np.array(angles).copy()
        self.cur_grip = int(gripper)

    def send_grip(self, gripper):
        if self.last_angles is None:
            raise ValueError('No arm position has been commanded yet.')
        self.write_joints(self.last_angles, gripper)


def main(args=None):
    rclpy.init(args=args)
    ik_node = IKNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(ik_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ik_node.stow_server.destroy()
        ik_node.pick_server.destroy()
        ik_node.place_server.destroy()
        ik_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
