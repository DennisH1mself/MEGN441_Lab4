from enum import Enum
import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosbot_arm.ik_node import inverse_kinematics
from rosbot_msgs.action import FindAndPick, Move
from rosbot_msgs.msg import BlockDetection
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException


class SequencerState(Enum):
    WAITING_FOR_INPUT = 0
    MOVING_TO_BLOCK_ESTIMATE = 1
    MOVING_TO_BLOCK_IN_SIGHT = 2
    PICKING_BLOCK = 3
    MOVING_TO_PLACE_POSITION = 4
    PLACING_BLOCK = 5
    DONE = 6
    FAILED = 7


class Sequencer(Node):
    def __init__(self):
        super().__init__('sequencer')
        self.state = SequencerState.WAITING_FOR_INPUT
        self.detection = None
        self.navigation_goal = None
        self.arm_goal = None
        self.delivery_goal = None
        self.target_color = None
        self.block_found = False
        self.block_in_range = False
        self.approach_block = None
        # Required for place/both: map [x, y, yaw] and arm [x, y, z, roll, pitch].
        self.declare_parameter('delivery_pose', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('place_target', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('pick_pitch', -45.0)
        self.declare_parameter('pickup_reach_margin', 0.05)  # Metres of spare forward reach.

        self.navigation_client = ActionClient(
            self, NavigateToPose, '/navigate')

        self.pick_client = ActionClient(
            self, Move, '/pick')

        self.place_client = ActionClient(
            self, Move, '/place')

        self.sequencer_server = ActionServer(
            self,
            FindAndPick,
            '/find_and_pick',
            execute_callback=self.find_and_pick_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscriber = self.create_subscription(
            BlockDetection, '/block_detector/block_detections',
            self.block_callback, 10)

        self.block_locations = {
            'red': None,
            'green': None,
            'blue': None,
        }
    def block_callback(self, msg):
        color = msg.color.lower()
        if color in self.block_locations:
            self.block_locations[color] = msg
            self.get_logger().info(
                f'Received {color} block at '
                f'({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})',
                throttle_duration_sec=1.0)
            self.navigation_feedback_callback(None)

    def goal_callback(self, request):
        if (self.navigation_goal is not None or self.arm_goal is not None or
                (self.delivery_goal is not None and self.delivery_goal.is_active)):
            return GoalResponse.REJECT
        if (self.delivery_goal is None and self.state not in (
                SequencerState.WAITING_FOR_INPUT, SequencerState.DONE,
                SequencerState.FAILED)):
            return GoalResponse.REJECT
        mode = (request.mode or 'both').lower()
        if mode not in ('pick', 'place', 'both'):
            return GoalResponse.REJECT
        color = request.block_color.lower()
        if mode != 'place':
            if (color not in self.block_locations or
                    not all(math.isfinite(v) for v in (
                        request.estimated_x, request.estimated_y))):
                return GoalResponse.REJECT

        self.target_color = color if mode != 'place' else None
        self.block_locations = {color: None for color in self.block_locations}
        self.block_found = False
        self.approach_block = None
        self.delivery_goal = None
        self.state = (SequencerState.MOVING_TO_PLACE_POSITION if mode == 'place'
                      else SequencerState.MOVING_TO_BLOCK_ESTIMATE)
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        for active_goal in (self.navigation_goal, self.arm_goal):
            if active_goal is not None:
                active_goal.cancel_goal_async()
        self.publish_feedback(goal_handle, 'Cancellation requested')
        return CancelResponse.ACCEPT

    async def find_and_pick_callback(self, goal_handle):
        self.delivery_goal = goal_handle
        mode = (goal_handle.request.mode or 'both').lower()
        result = FindAndPick.Result()
        try:
            if goal_handle.is_cancel_requested:
                self.state = SequencerState.WAITING_FOR_INPUT
                goal_handle.canceled()
                return result
            if mode not in ('pick', 'place', 'both'):
                raise ValueError('mode must be pick, place, or both')

            # Check the destination before picking up a block for delivery.
            if mode in ('place', 'both'):
                pose = self.get_parameter_or('delivery_pose').value
                if (pose is None or len(pose) != 3 or
                        not all(math.isfinite(v) for v in pose)):
                    raise ValueError('Set delivery_pose to map [x, y, yaw radians]')
                target = self.get_parameter_or('place_target').value
                if (target is None or len(target) != 5 or
                        not all(math.isfinite(v) for v in target)):
                    raise ValueError('Set place_target to arm [x, y, z, roll, pitch degrees]')

            success = True
            if mode in ('pick', 'both'):
                success = await self.move_to_block_estimate(goal_handle)
                if success and not goal_handle.is_cancel_requested:
                    success = await self.move_to_block_in_sight(goal_handle)
                if success and not goal_handle.is_cancel_requested:
                    success = await self.pick_block(goal_handle)

            if (success and mode in ('place', 'both') and
                    not goal_handle.is_cancel_requested):
                success = await self.move_to_place_position(goal_handle)
                if success and not goal_handle.is_cancel_requested:
                    success = await self.place_block(goal_handle)

            if goal_handle.is_cancel_requested:
                self.state = SequencerState.WAITING_FOR_INPUT
                self.publish_feedback(goal_handle, 'Operation canceled')
                goal_handle.canceled()
            elif not success:
                self.state = SequencerState.FAILED
                goal_handle.abort()
                result.error_occured = True
            else:
                self.state = SequencerState.DONE
                status_msg = ('Pickup complete; arm stowed' if mode == 'pick'
                              else 'Delivery complete; arm stowed')
                self.publish_feedback(goal_handle, status_msg)
                goal_handle.succeed()
                result.succeeded = True
        except Exception as error:
            self.get_logger().error(str(error))
            self.state = SequencerState.FAILED
            self.publish_feedback(goal_handle, str(error))
            goal_handle.abort()
            result.error_occured = True
        finally:
            self.delivery_goal = None
        return result

    async def move_to_block_estimate(self, goal_handle):
        # Stop estimate navigation when a recent matching block is detected.
        self.delivery_goal = goal_handle
        self.state = SequencerState.MOVING_TO_BLOCK_ESTIMATE
        estimated_x = goal_handle.request.estimated_x
        estimated_y = goal_handle.request.estimated_y
        self.target_color = goal_handle.request.block_color.lower()
        self.block_found = False
        self.navigation_goal = None
        try:
            if goal_handle.is_cancel_requested:
                return False
            if not self.navigation_client.server_is_ready():
                raise RuntimeError('Navigation server is not ready')
            self.publish_feedback(goal_handle, 'Moving toward the pickup estimate')

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose.header.frame_id = 'map'
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
            nav_goal.pose.pose.position.x = estimated_x
            nav_goal.pose.pose.position.y = estimated_y
            nav_goal.pose.pose.orientation.w = 1.0  # Example: yaw = 0.
            nav_handler = await self.navigation_client.send_goal_async(
                nav_goal,
                feedback_callback=self.navigation_feedback_callback,
            )

            if not nav_handler.accepted:
                raise RuntimeError("Navigation goal was rejected")

            self.navigation_goal = nav_handler
            if goal_handle.is_cancel_requested:
                nav_handler.cancel_goal_async()
            # A detection may have arrived before the goal was accepted.
            self.navigation_feedback_callback(None)
            nav_response = await nav_handler.get_result_async()
            if goal_handle.is_cancel_requested:
                return False
            if self.block_found and nav_response.status in (
                    GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                # Wait for navigation to finish stopping before advancing.
                self.state = SequencerState.MOVING_TO_BLOCK_IN_SIGHT
                self.publish_feedback(goal_handle, 'Block located; estimate navigation stopped')
                self.get_logger().info('Block located; estimate navigation stopped')
                return True
            if nav_response.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError('Navigation not successful')
            raise RuntimeError('Reached the estimate without locating the block')
        except Exception as e:
            self.get_logger().error(str(e))
            self.state = SequencerState.FAILED
            self.publish_feedback(goal_handle, str(e))
            return False
        finally:
            self.navigation_goal = None

    async def move_to_block_in_sight(self, goal_handle):
        self.delivery_goal = goal_handle
        self.state = SequencerState.MOVING_TO_BLOCK_IN_SIGHT
        self.target_color = goal_handle.request.block_color.lower()
        self.block_in_range = False
        self.approach_block = None
        try:
            if goal_handle.is_cancel_requested:
                return False
            if not self.navigation_client.server_is_ready():
                raise RuntimeError('Navigation server is not ready')

            color = goal_handle.request.block_color.lower()
            detection = self.block_locations.get(color)
            if detection is None:
                raise RuntimeError(f'No {color} block detection available')
            image_time = Time.from_msg(detection.header.stamp)
            age = (self.get_clock().now() - image_time).nanoseconds / 1e9
            if not 0.0 <= age <= 1.0:
                raise RuntimeError('Block detection is stale')

            point = PointStamped()
            point.header, point.point = detection.header, detection.position
            transform = self.tf_buffer.lookup_transform(
                'map', detection.header.frame_id, image_time)
            block = do_transform_point(point, transform).point
            if not all(math.isfinite(v) for v in (block.x, block.y, block.z)):
                raise RuntimeError('Block position is invalid')
            if self.get_block_position('odom') is None:
                raise RuntimeError('Cannot track the detected block in odom')

            robot = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time()).transform.translation
            yaw = math.atan2(block.y - robot.y, block.x - robot.x)
            stand_off = 0.25 - self.pickup_reach_margin

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose.header.frame_id = 'map'
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
            nav_goal.pose.pose.position.x = block.x - stand_off * math.cos(yaw)
            nav_goal.pose.pose.position.y = block.y - stand_off * math.sin(yaw)
            nav_goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            nav_goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            # Plan once: replanning this short approach can create a large loop.
            nav_goal.behavior_tree = os.path.join(
                get_package_share_directory('delivery_sequencer'),
                'behavior_trees', 'approach_block.xml')

            nav_handle = await self.navigation_client.send_goal_async(
                nav_goal, feedback_callback=self.navigation_feedback_callback)
            if not nav_handle.accepted:
                raise RuntimeError('Approach navigation was rejected')

            self.navigation_goal = nav_handle
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()
            self.publish_feedback(goal_handle, 'Approaching the detected block')
            self.navigation_feedback_callback(None)
            response = await nav_handle.get_result_async()
            if goal_handle.is_cancel_requested:
                return False
            if (response.status != GoalStatus.STATUS_SUCCEEDED and not (
                    response.status == GoalStatus.STATUS_CANCELED and
                    self.block_in_range)):
                raise RuntimeError('Approach navigation failed')

            # Navigation arriving does not guarantee that the arm can reach.
            self.get_pick_target()
            self.state = SequencerState.PICKING_BLOCK
            return True
        except Exception as error:
            self.get_logger().error(str(error))
            self.state = SequencerState.FAILED
            self.publish_feedback(goal_handle, str(error))
            return False
        finally:
            self.navigation_goal = None

    async def pick_block(self, goal_handle):
        self.delivery_goal = goal_handle
        self.state = SequencerState.PICKING_BLOCK
        self.target_color = goal_handle.request.block_color.lower()
        try:
            if goal_handle.is_cancel_requested:
                return False
            if not self.pick_client.server_is_ready():
                raise RuntimeError('Pick server is not ready')
            target = self.get_pick_target()
            self.publish_feedback(goal_handle, 'Picking the block')
            arm_handle = await self.pick_client.send_goal_async(
                target, feedback_callback=self.pick_feedback_callback)
            if not arm_handle.accepted:
                raise RuntimeError('Pick goal was rejected')

            self.arm_goal = arm_handle
            if goal_handle.is_cancel_requested:
                arm_handle.cancel_goal_async()
            response = await arm_handle.get_result_async()
            if goal_handle.is_cancel_requested:
                return False
            if (response.status != GoalStatus.STATUS_SUCCEEDED or
                    not response.result.succeeded or response.result.error_occured):
                raise RuntimeError('Pick failed')

            self.state = SequencerState.MOVING_TO_PLACE_POSITION
            self.publish_feedback(goal_handle, 'Block picked; arm stowed')
            return True
        except Exception as error:
            self.get_logger().error(str(error))
            self.state = SequencerState.FAILED
            self.publish_feedback(goal_handle, str(error))
            return False
        finally:
            self.arm_goal = None

    async def move_to_place_position(self, goal_handle):
        self.delivery_goal = goal_handle
        self.state = SequencerState.MOVING_TO_PLACE_POSITION
        try:
            if goal_handle.is_cancel_requested:
                return False
            pose = self.get_parameter_or('delivery_pose').value
            if (pose is None or len(pose) != 3 or
                    not all(math.isfinite(v) for v in pose)):
                raise ValueError('Set delivery_pose to map [x, y, yaw radians]')
            if not self.navigation_client.server_is_ready():
                raise RuntimeError('Navigation server is not ready')

            x, y, yaw = pose
            target = NavigateToPose.Goal()
            target.pose.header.frame_id = 'map'
            target.pose.header.stamp = self.get_clock().now().to_msg()
            target.pose.pose.position.x = x
            target.pose.pose.position.y = y
            target.pose.pose.orientation.z = math.sin(yaw / 2.0)
            target.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.publish_feedback(goal_handle, 'Navigating to the delivery position')
            nav_handle = await self.navigation_client.send_goal_async(
                target, feedback_callback=self.navigation_feedback_callback)
            if not nav_handle.accepted:
                raise RuntimeError('Delivery navigation was rejected')

            self.navigation_goal = nav_handle
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()
            response = await nav_handle.get_result_async()
            if goal_handle.is_cancel_requested:
                return False
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError('Delivery navigation failed')

            self.state = SequencerState.PLACING_BLOCK
            self.publish_feedback(goal_handle, 'Reached the delivery position')
            return True
        except Exception as error:
            self.get_logger().error(str(error))
            self.state = SequencerState.FAILED
            self.publish_feedback(goal_handle, str(error))
            return False
        finally:
            self.navigation_goal = None

    async def place_block(self, goal_handle):
        self.delivery_goal = goal_handle
        self.state = SequencerState.PLACING_BLOCK
        try:
            if goal_handle.is_cancel_requested:
                return False
            pose = self.get_parameter_or('place_target').value
            if (pose is None or len(pose) != 5 or
                    not all(math.isfinite(v) for v in pose)):
                raise ValueError('Set place_target to arm [x, y, z, roll, pitch degrees]')
            if not self.place_client.server_is_ready():
                raise RuntimeError('Place server is not ready')

            x, y, z, roll, pitch = pose
            target = Move.Goal(x=x, y=y, z=z, roll=roll, pitch=pitch)
            self.publish_feedback(goal_handle, 'Placing the block')
            arm_handle = await self.place_client.send_goal_async(
                target, feedback_callback=self.place_feedback_callback)
            if not arm_handle.accepted:
                raise RuntimeError('Place goal was rejected')

            self.arm_goal = arm_handle
            if goal_handle.is_cancel_requested:
                arm_handle.cancel_goal_async()
            response = await arm_handle.get_result_async()
            if goal_handle.is_cancel_requested:
                return False
            if (response.status != GoalStatus.STATUS_SUCCEEDED or
                    not response.result.succeeded or response.result.error_occured):
                raise RuntimeError('Place failed')

            self.state = SequencerState.DONE
            self.publish_feedback(goal_handle, 'Block placed; arm stowed')
            return True
        except Exception as error:
            self.get_logger().error(str(error))
            self.state = SequencerState.FAILED
            self.publish_feedback(goal_handle, str(error))
            return False
        finally:
            self.arm_goal = None

    @property
    def pickup_reach_margin(self):
        margin = self.get_parameter('pickup_reach_margin').value
        if not 0.0 <= margin < 0.25:
            raise ValueError('pickup_reach_margin must be in [0, 0.25) metres')
        return margin

    def get_pick_target(self):
        block = self.get_block_position('base_footprint')
        if block is None:
            raise ValueError('No recent block position available for picking')
        # Use the same calibrated target for the reach check and the pick.
        target = Move.Goal(
            x=block.point.x, y=block.point.y + 0.02,
            z=block.point.z - 0.01, roll=0.0,
            pitch=float(self.get_parameter('pick_pitch').value))
        try:
            inverse_kinematics(target)
        except ValueError as error:
            raise ValueError(
                f'{error} Block target in base_footprint: '
                f'x={target.x:.3f}, y={target.y:.3f}, z={target.z:.3f}, '
                f'pitch={target.pitch:.1f}') from error
        # Keep approaching until the arm has spare reach. Only the measured
        # block target below is sent to the arm.
        margin = self.pickup_reach_margin
        if margin:
            try:
                inverse_kinematics(Move.Goal(
                    x=target.x + margin, y=target.y, z=target.z,
                    roll=target.roll, pitch=target.pitch))
            except ValueError as error:
                raise ValueError(f'Approaching for {margin:.2f} m of spare arm reach') from error
        return target

    def get_block_position(self, target_frame):
        detection = self.block_locations.get(self.target_color)
        tracking = self.state in (SequencerState.MOVING_TO_BLOCK_IN_SIGHT,
                                  SequencerState.PICKING_BLOCK)
        try:
            now = self.get_clock().now()
            fresh = (detection is not None and
                     detection.color.lower() == self.target_color and
                     0.0 <= (now - Time.from_msg(detection.header.stamp)).nanoseconds / 1e9 <= 1.0)
            if fresh:
                point = PointStamped()
                point.header, point.point = detection.header, detection.position
                image_time = Time.from_msg(detection.header.stamp)
                if tracking:
                    # Save a stationary point; never reuse old camera-relative XYZ.
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            'odom', detection.header.frame_id, image_time)
                        point = do_transform_point(point, transform)
                        if not all(math.isfinite(v) for v in (
                                point.point.x, point.point.y, point.point.z)):
                            return None
                        point.header.stamp = detection.header.stamp
                        self.approach_block = point
                    except TransformException:
                        if self.approach_block is None:
                            raise
            elif tracking and self.approach_block is not None:
                point = self.approach_block
            else:
                return None

            # During pickup, account for movement since the camera observation.
            if tracking:
                point = self.approach_block
                age = (now - Time.from_msg(point.header.stamp)).nanoseconds / 1e9
                if not 0.0 <= age <= 5.0:
                    return None
                image_time = Time()
            transform = self.tf_buffer.lookup_transform(
                target_frame, point.header.frame_id, image_time)
            transformed = do_transform_point(point, transform)
            if not all(math.isfinite(v) for v in (
                    transformed.point.x, transformed.point.y, transformed.point.z)):
                return None
            return transformed
        except TransformException as error:
            self.get_logger().warn(f'Waiting for block TF: {error}',
                                   throttle_duration_sec=5.0)
            return None

    def navigation_feedback_callback(self, feedback_msg):
        if feedback_msg is not None:
            status_msg = f'Distance remaining: {feedback_msg.feedback.distance_remaining:.2f} m'
            self.get_logger().info(
                status_msg, throttle_duration_sec=1.0)
            self.publish_feedback(self.delivery_goal, status_msg)

        if self.navigation_goal is None:
            return

        if self.state == SequencerState.MOVING_TO_BLOCK_IN_SIGHT:
            if self.block_in_range:
                return
            try:
                self.get_pick_target()
            except ValueError as error:
                self.get_logger().warn(str(error), throttle_duration_sec=2.0)
                if self.get_block_position('base_footprint') is None:
                    self.state = SequencerState.FAILED
                    self.publish_feedback(
                        self.delivery_goal, 'Lost block position; stopping approach')
                    self.navigation_goal.cancel_goal_async()
                return  # Keep approaching until IK accepts the actual target.
            self.block_in_range = True
            self.navigation_goal.cancel_goal_async()
            self.publish_feedback(
                self.delivery_goal, 'Block is within arm reach; stopping approach')
            return

        if (self.state != SequencerState.MOVING_TO_BLOCK_ESTIMATE or
                self.block_found):
            return

        detection = self.block_locations.get(self.target_color)
        if detection is None:
            return
        age = (self.get_clock().now() -
               Time.from_msg(detection.header.stamp)).nanoseconds / 1e9
        if not 0.0 <= age <= 1.0:
            return

        self.navigation_goal.cancel_goal_async()
        self.block_found = True  # Prevent repeated cancellation requests.
        self.get_logger().info(
            f'{self.target_color} block detected; canceling estimate navigation')

    def pick_feedback_callback(self, feedback_msg):
        status_msg = f'Pick: {feedback_msg.feedback.status}'
        self.get_logger().info(status_msg, throttle_duration_sec=1.0)
        self.publish_feedback(self.delivery_goal, status_msg)

    def place_feedback_callback(self, feedback_msg):
        status_msg = f'Place: {feedback_msg.feedback.status}'
        self.get_logger().info(status_msg, throttle_duration_sec=1.0)
        self.publish_feedback(self.delivery_goal, status_msg)

    def publish_feedback(self, goal_handle, status_msg):
        if goal_handle is not None and goal_handle.is_active:
            goal_handle.publish_feedback(FindAndPick.Feedback(
                status=self.state.value, status_msg=status_msg))


def main():
    rclpy.init()
    node = Sequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
