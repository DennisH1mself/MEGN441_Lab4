from enum import Enum, auto
import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, Empty
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from rosbot_arm.ik_node import STOW_POSE, inverse_kinematics
from rosbot_msgs.action import Move
from rosbot_msgs.msg import BlockDetection, IKTarget


class State(Enum):
    IDLE = auto()
    LOCALIZING = auto()
    STOWING = auto()
    CHECKING_INITIAL_STOW = auto()
    DRIVING_TO_ESTIMATE = auto()
    WAITING_FOR_DETECTION = auto()
    CANCELING_ESTIMATE = auto()
    APPROACHING_BLOCK = auto()
    CANCELING_APPROACH = auto()
    STOPPING_AT_BLOCK = auto()
    FINDING_GRASP = auto()
    PICKING = auto()
    CHECKING_PICK_STOW = auto()
    DRIVING_TO_DROPOFF = auto()
    CANCELING_DROPOFF = auto()
    STOPPING_AT_DROPOFF = auto()
    PLACING = auto()
    CHECKING_FINAL_STOW = auto()
    DONE = auto()
    FAILED = auto()


class Sequencer(Node):
    def __init__(self):
        super().__init__('sequencer')
        self.start_pose = self.read_numbers('start_pose', [math.nan] * 3)
        self.estimated_block = self.read_numbers('estimated_block', [math.nan] * 2)
        self.approach_yaw = self.read_number('approach_yaw_deg', math.nan)
        self.viewing_distance = self.read_number('viewing_distance_m', 0.65)
        self.detection_window = self.read_number('detection_window_m', 0.65)
        self.localization_timeout = self.read_number('localization_timeout_s', 25.0)
        self.stop_timeout = self.read_number('stop_timeout_s', 8.0)
        self.retry_standoffs = self.read_numbers('refined_approach_retry_m', [0.30, 0.32])
        self.approach_attempt = 0
        self.dropoff = self.read_numbers('dropoff_pose', [math.nan] * 3)
        place = self.read_numbers('place_pose', [math.nan] * 5)
        self.place_goal = Move.Goal(**dict(zip(('x', 'y', 'z', 'pitch', 'roll'), place)))
        self.grasp_offset = self.read_numbers('grasp_offset', [0.0] * 3)
        self.stand_off = self.read_number('stand_off_m', 0.26)
        if not 0 < self.stand_off < self.viewing_distance or self.detection_window <= 0:
            raise ValueError('Require 0 < stand_off_m < viewing_distance_m and detection_window_m > 0')
        if any(not 0 < d < self.viewing_distance for d in self.retry_standoffs):
            raise ValueError('Refined approach retry stand-offs must be positive and below viewing distance')
        self.pitch = self.read_number('pick_pitch_deg', -45.0)
        self.roll = self.read_number('pick_roll_deg', 0.0)
        self.color = self.declare_parameter('block_color', 'red').value.strip().lower()
        if self.color not in ('red', 'green', 'blue'):
            raise ValueError('block_color must be red, green, or blue')
        inverse_kinematics(self.place_goal)
        self.stow_angles = [round(float(angle)) for angle in inverse_kinematics(IKTarget(**STOW_POSE))]

        self.nav = ActionClient(self, NavigateToPose, '/navigate')
        self.nav2 = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.stow = ActionClient(self, Move, '/stow')
        self.pick = ActionClient(self, Move, '/pick')
        self.place = ActionClient(self, Move, '/place')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            BlockDetection, '/block_detector/block_detections', self.on_detection, 10
        )
        self.create_subscription(
            JointState, '/controller_manager/joint_states', self.on_joint_state, 10
        )
        self.create_subscription(Odometry, '/odom', self.on_odometry, qos_profile_sensor_data)

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.amcl_received_ns = 0
        self.last_pose_request_ns = 0
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.on_navigation_status, qos_profile_action_status_default,
        )

        self.detection = None
        self.amcl_pose = None
        self.navigation_active = False
        self.block_in_map = None
        self.joints = None
        self.odometry = None
        self.stopped_since = None
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_sent = False
        self.completed_response = None
        self.action_error = None
        self.goal_generation = 0
        self.state = State.IDLE
        self.timer = self.create_timer(0.1, self.update)
        self.create_service(Trigger, '/start_delivery', self.start_delivery)
        self.create_service(Trigger, '/stop_delivery', self.stop_delivery)
        self.get_logger().info('Idle. Call /start_delivery when the robot is at the configured start.')

    def read_numbers(self, name, defaults):
        values = self.declare_parameter(name, defaults).value
        if len(values) != len(defaults) or not all(math.isfinite(value) for value in values):
            raise ValueError(f'Set {name} to {len(defaults)} finite numbers in delivery.yaml')
        return values

    def read_number(self, name, default):
        value = self.declare_parameter(name, default).value
        if not math.isfinite(value):
            raise ValueError(f'Set {name} to a finite number in delivery.yaml')
        return value

    def start_delivery(self, request, response):
        if self.state != State.IDLE:
            response.message = 'This sequencer runs one delivery. Restart it for a new run.'
        elif self.navigation_active:
            response.message = 'Cancel the active Nav2 goal first.'
        elif not all(client.server_is_ready() for client in (
            self.nav, self.nav2, self.stow, self.pick, self.place,
        )):
            response.message = 'Waiting for Nav2, nav_manager, or IK action servers.'
        elif not self.fresh(self.odometry) or not self.base_stopped(0):
            response.message = 'Waiting for fresh odometry from a stationary base.'
        elif self.initial_pose_pub.get_subscription_count() == 0:
            response.message = 'Waiting for AMCL to subscribe to /initialpose.'
        else:
            self.transition(State.LOCALIZING, self.localization_timeout)
            self.publish_initial_pose()
            self.last_pose_request_ns = self.get_clock().now().nanoseconds
            self.request_nomotion_update()
            response.success = True
            response.message = 'Delivery started. Waiting for localization before stowing.'
        return response

    def stop_delivery(self, request, response):
        if self.state in (State.IDLE, State.DONE, State.FAILED):
            response.message = 'No delivery is running.'
        else:
            self.fail('Stop requested by operator')
            response.success = True
            response.message = 'Sequence stopped. Cancellation requested for any active goal.'
        return response

    def publish_initial_pose(self):
        x, y, yaw_deg = self.start_pose
        yaw = math.radians(yaw_deg)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(15.0) ** 2
        self.initial_pose_pub.publish(msg)

    def on_amcl_pose(self, msg):
        self.amcl_pose = msg
        # AMCL can publish a measurement stamp slightly behind the receive time.
        # Fresh means received after start, not only header.stamp > state entry.
        self.amcl_received_ns = self.get_clock().now().nanoseconds

    def request_nomotion_update(self):
        if self.nomotion_client.service_is_ready():
            self.nomotion_client.call_async(Empty.Request())

    def on_navigation_status(self, msg):
        self.navigation_active = any(status.status in (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        ) for status in msg.status_list)

    def on_detection(self, msg):
        if msg.color.lower() == self.color:
            self.detection = msg

    def on_joint_state(self, msg):
        self.joints = msg

    def on_odometry(self, msg):
        self.odometry = msg

    def transition(self, state, timeout):
        self.state = state
        self.entered_at = self.get_clock().now().nanoseconds
        self.deadline = time.monotonic() + timeout
        self.stopped_since = None
        self.get_logger().info(state.name)

    # ACTION CLIENT: same pattern as the turtlesim waypoint client.
    # send_goal_async -> goal_response_callback -> result_callback.
    def send(self, state, client, request, timeout):
        if not client.server_is_ready():
            raise RuntimeError(f'Action server for {state.name} is unavailable')

        self.transition(state, timeout)
        self.goal_generation += 1
        generation = self.goal_generation
        self.goal_handle = None
        self.result_future = None
        self.completed_response = None
        self.action_error = None
        self.cancel_sent = False

        self.get_logger().info(f'Sending action goal for {state.name}')
        self.goal_future = client.send_goal_async(
            request, feedback_callback=self.feedback_callback)
        self.goal_future.add_done_callback(
            lambda future, g=generation: self.goal_response_callback(future, g))

    def feedback_callback(self, feedback_msg):
        # Move.Feedback contains status/in_range/busy; Nav2 feedback has
        # other fields. Only print recognized feedback fields.
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'status'):
            self.get_logger().info(f'Arm feedback: {feedback.status}',
                                   throttle_duration_sec=1.0)

    def goal_response_callback(self, future, generation):
        # This runs when the action server ACCEPTS or REJECTS the goal.
        try:
            handle = future.result()
            if generation != self.goal_generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.action_error = 'Action goal rejected by server'
                return
            self.goal_handle = handle
            self.result_future = handle.get_result_async()
            self.result_future.add_done_callback(
                lambda f, g=generation: self.result_callback(f, g))
            if self.state == State.FAILED or self.state in (
                    State.CANCELING_ESTIMATE, State.CANCELING_APPROACH,
                    State.CANCELING_DROPOFF):
                self.request_cancel()
        except Exception as error:
            self.action_error = f'Unable to accept action goal: {error}'

    def result_callback(self, future, generation):
        # This runs when the action actually FINISHES (or is canceled).
        if generation != self.goal_generation:
            return
        try:
            self.completed_response = future.result()
            self.get_logger().info(
                f'Action completed, ROS status={self.completed_response.status}')
        except Exception as error:
            self.action_error = f'Unable to read action result: {error}'

    def request_cancel(self):
        # Cancellation is asynchronous; continue waiting for the final result.
        if self.goal_handle is not None and not self.cancel_sent:
            self.goal_handle.cancel_goal_async()
            self.cancel_sent = True

    def poll_result(self, cancel=False):
        # The timer-driven state machine consumes results produced by the
        # callbacks. It does not block or poll Future.done() repeatedly.
        if self.action_error:
            raise RuntimeError(self.action_error)
        if cancel:
            self.request_cancel()
        if self.completed_response is None:
            return None
        response = self.completed_response
        self.completed_response = None
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        return response

    def succeeded(self, arm=False):
        response = self.poll_result()
        if response is None:
            return False
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Action ended with status {response.status}')
        if arm and (not response.result.succeeded or response.result.error_occured):
            raise RuntimeError('Arm action reported failure')
        return True

    def navigation_goal(self, pose):
        x, y, yaw_deg = pose
        yaw = math.radians(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def fresh(self, msg, after=0):
        if msg is None:
            return False
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        age = self.get_clock().now().nanoseconds - stamp
        return stamp > after and 0 <= age < 750_000_000

    def stow_confirmed(self):
        if not self.fresh(self.joints, self.entered_at):
            return False
        positions = dict(zip(self.joints.name, self.joints.position))
        for servo_id, expected in enumerate(self.stow_angles, start=1):
            measured = positions.get(f'joint{servo_id}', math.nan)
            # arm_control reports URDF radians with the opposite servo sign.
            if not math.isfinite(measured) or abs(-math.degrees(measured) - expected) > 4.0:
                return False
        return True

    def base_stopped(self, after):
        if self.fresh(self.odometry, after):
            twist = self.odometry.twist.twist
            if math.hypot(twist.linear.x, twist.linear.y) < 0.02 and abs(twist.angular.z) < 0.05:
                sample_time = Time.from_msg(self.odometry.header.stamp).nanoseconds
                if self.stopped_since is None or sample_time < self.stopped_since:
                    self.stopped_since = sample_time
                return sample_time - self.stopped_since >= 300_000_000
        self.stopped_since = None
        return False

    def transform_detection(self, frame, after=0):
        msg = self.detection
        if not self.fresh(msg, after):
            return None
        point = PointStamped(header=msg.header, point=msg.position)
        try:
            transform = self.tf_buffer.lookup_transform(
                frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)
            )
        except TransformException:
            return None
        point = do_transform_point(point, transform).point
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return None
        return point

    def pose_near(self, pose, distance_tolerance, yaw_tolerance):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return False
        if not self.fresh(transform):
            return False
        position = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
        )
        x, y, yaw_deg = pose
        yaw_error = math.remainder(yaw - math.radians(yaw_deg), 2.0 * math.pi)
        return (
            math.hypot(position.x - x, position.y - y) <= distance_tolerance
            and abs(yaw_error) <= yaw_tolerance
        )

    def approach_pose(self, x, y, distance):
        yaw = math.radians(self.approach_yaw)
        return x - distance * math.cos(yaw), y - distance * math.sin(yaw), self.approach_yaw

    def find_block(self, after=0):
        point = self.transform_detection('map', after)
        if point is None:
            return None
        x, y = self.estimated_block
        if math.hypot(point.x - x, point.y - y) > self.detection_window:
            return None
        return point

    def approach_block(self, distance=None):
        if distance is None:
            distance = self.stand_off
            self.approach_attempt = 0
        pose = self.approach_pose(self.block_in_map.x, self.block_in_map.y, distance)
        self.get_logger().info(
            f'Camera-refined approach, stand-off={distance:.2f}m, ' 
            f'map goal=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.1f}deg)')
        self.send(State.APPROACHING_BLOCK, self.nav, self.navigation_goal(pose), 90.0)

    def navigation_cancelled(self):
        response = self.poll_result(cancel=True)
        if response is None:
            return False
        if response.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
            raise RuntimeError(f'Canceled navigation ended with status {response.status}')
        return True

    def detected_grasp(self, after):
        mapped = self.transform_detection('map', after)
        if mapped is None or math.hypot(
            mapped.x - self.block_in_map.x, mapped.y - self.block_in_map.y
        ) > 0.20:
            return None
        block = self.transform_detection('base_footprint', after)
        if block is None:
            return None
        dx, dy, dz = self.grasp_offset
        # The optical point must be in front and near the arm centerline.
        if not 0.12 <= block.x + dx <= 0.34 or abs(block.y + dy) > 0.08:
            return None
        goal = Move.Goal(
            x=block.x + dx, y=block.y + dy, z=block.z + dz,
            pitch=self.pitch, roll=self.roll,
        )
        try:
            inverse_kinematics(goal)
        except ValueError as error:
            self.get_logger().warning(f'Detected block is outside grasp limits: {error}',
                                      throttle_duration_sec=1.0)
            return None
        return goal

    def update(self):
        if self.state == State.IDLE:
            self.base_stopped(0)
            return
        try:
            if time.monotonic() >= self.deadline:
                raise RuntimeError(
                    f'Timed out in {self.state.name}. Check AMCL/TF/odom, ' 
                    f'camera detection, arm joint feedback, or action logs for this state')

            if self.state == State.LOCALIZING:
                if self.navigation_active:
                    raise RuntimeError('A separate Nav2 goal became active during setup')
                if self.get_clock().now().nanoseconds - self.last_pose_request_ns > 3_000_000_000:
                    self.publish_initial_pose()
                    self.request_nomotion_update()
                    self.last_pose_request_ns = self.get_clock().now().nanoseconds
                if (self.amcl_pose is not None
                        and self.amcl_received_ns > self.entered_at
                        and self.pose_near(self.start_pose, 0.35, 0.45)
                        and self.base_stopped(0)):
                    self.send(State.STOWING, self.stow, Move.Goal(), 15.0)

            elif self.state == State.STOWING:
                if self.succeeded(arm=True):
                    self.transition(State.CHECKING_INITIAL_STOW, 5.0)

            elif self.state == State.CHECKING_INITIAL_STOW:
                if self.stow_confirmed():
                    coarse = self.approach_pose(*self.estimated_block, self.viewing_distance)
                    self.send(State.DRIVING_TO_ESTIMATE, self.nav,
                              self.navigation_goal(coarse), 90.0)

            elif self.state == State.DRIVING_TO_ESTIMATE:
                point = self.find_block(self.entered_at)
                if point is not None:
                    self.block_in_map = point
                    self.transition(State.CANCELING_ESTIMATE, 10.0)
                elif self.succeeded():
                    self.transition(State.WAITING_FOR_DETECTION, 8.0)

            elif self.state == State.WAITING_FOR_DETECTION:
                point = self.find_block()
                if point is not None:
                    self.block_in_map = point
                    self.approach_block()

            elif self.state == State.CANCELING_ESTIMATE:
                if self.navigation_cancelled():
                    self.approach_block()

            elif self.state == State.APPROACHING_BLOCK:
                if self.detected_grasp(self.entered_at) is not None:
                    self.transition(State.CANCELING_APPROACH, 10.0)
                else:
                    # Inspect result so a no-path ABORT can retry from a safer
                    # stand-off rather than killing the mission immediately.
                    response = self.poll_result()
                    if response is not None:
                        if response.status == GoalStatus.STATUS_SUCCEEDED:
                            self.transition(State.STOPPING_AT_BLOCK, self.stop_timeout)
                        elif response.status == GoalStatus.STATUS_ABORTED and self.approach_attempt < len(self.retry_standoffs):
                            distance = self.retry_standoffs[self.approach_attempt]
                            self.approach_attempt += 1
                            self.get_logger().warning(
                                f'Nav2 aborted refined approach. Retry {self.approach_attempt} ' 
                                f'at stand-off {distance:.2f}m. Check costmap if this fails.')
                            self.approach_block(distance)
                        else:
                            raise RuntimeError(
                                f'Refined navigation failed with ROS status {response.status}; ' 
                                'check planner_server/controller_server logs and RViz costmap')

            elif self.state == State.CANCELING_APPROACH:
                if self.navigation_cancelled():
                    self.transition(State.STOPPING_AT_BLOCK, self.stop_timeout)

            elif self.state == State.STOPPING_AT_BLOCK:
                if self.base_stopped(self.entered_at):
                    self.transition(State.FINDING_GRASP, 8.0)

            elif self.state == State.FINDING_GRASP:
                goal = self.detected_grasp(self.entered_at)
                if goal is not None and self.base_stopped(self.entered_at):
                    self.get_logger().info(
                        f'Picking detected {self.color} block at '
                        f'({goal.x:.3f}, {goal.y:.3f}, {goal.z:.3f}) in base_footprint'
                    )
                    self.send(State.PICKING, self.pick, goal, 20.0)

            elif self.state == State.PICKING:
                if self.succeeded(arm=True):
                    self.transition(State.CHECKING_PICK_STOW, 5.0)

            elif self.state == State.CHECKING_PICK_STOW:
                if self.stow_confirmed():
                    self.send(State.DRIVING_TO_DROPOFF, self.nav,
                              self.navigation_goal(self.dropoff), 90.0)

            elif self.state == State.DRIVING_TO_DROPOFF:
                if self.pose_near(self.dropoff, 0.05, 0.15):
                    self.transition(State.CANCELING_DROPOFF, 10.0)
                elif self.succeeded():
                    self.transition(State.STOPPING_AT_DROPOFF, self.stop_timeout)

            elif self.state == State.CANCELING_DROPOFF:
                if self.navigation_cancelled():
                    self.transition(State.STOPPING_AT_DROPOFF, self.stop_timeout)

            elif self.state == State.STOPPING_AT_DROPOFF:
                if self.base_stopped(self.entered_at):
                    if not self.pose_near(self.dropoff, 0.07, 0.20):
                        raise RuntimeError('Base stopped outside the drop-off pose tolerance')
                    self.send(State.PLACING, self.place, self.place_goal, 20.0)

            elif self.state == State.PLACING:
                if self.succeeded(arm=True):
                    self.transition(State.CHECKING_FINAL_STOW, 5.0)

            elif self.state == State.CHECKING_FINAL_STOW:
                if self.stow_confirmed():
                    self.state = State.DONE
                    self.timer.cancel()
                    self.get_logger().info('Delivery complete. Arm is stowed.')

        except Exception as error:
            self.fail(str(error))

    def fail(self, reason):
        self.get_logger().error(f'Delivery stopped in {self.state.name}: {reason}')
        self.state = State.FAILED
        self.timer.cancel()
        self.request_cancel()
        # If acceptance is still pending, goal_response_callback() will
        # cancel the accepted goal when its future completes.


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Sequencer()
        rclpy.spin(node)
    except ValueError as error:
        print(f'Invalid delivery configuration: {error}')
        raise SystemExit(1) from None
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
