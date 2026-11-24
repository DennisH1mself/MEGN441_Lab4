import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros_robot_controller_msgs.msg import BusServoState, SetBusServoState
from rosbot_msgs.msg import JointControl
from rosbot_msgs.srv import GetJointAngle

class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')
        # The base controller owns the serial port and also receives the IMU.
        self.servo_pub = self.create_publisher(
            SetBusServoState, '/ros_robot_controller/bus_servo/set_state', 10)
        self.subscriber = self.create_subscription(JointControl, '/joint_control', self.arm_control_callback, 10)
        self.service = self.create_service(GetJointAngle, '/get_joint_angle', self.get_joint_angle_callback)
        self.joint_state_pub = self.create_publisher(JointState, '/controller_manager/joint_states', 10)
        self._joint_state = None
        # Reuse measured feedback; servo reads in the base executor can block driving.
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn('Incomplete joint feedback', throttle_duration_sec=5.0)
            return
        self._joint_state = msg
        self.joint_state_pub.publish(msg)

    def get_joint_angle_callback(self, request, response):
        joint_name = f'joint{request.joint_id}'
        try:
            index = self._joint_state.name.index(joint_name)
            # Factory joint states use the opposite sign to servo angles.
            response.angle = float(round(-math.degrees(self._joint_state.position[index])))
        except (AttributeError, ValueError, IndexError):
            self.warn(f'No joint feedback available for {joint_name}')
            response.angle = math.nan
        return response
    def warn(self, msg):
        self.get_logger().warn(msg)
    def arm_control_callback(self, msg):
        angles = [
            msg.rotation,
            msg.joint1,
            msg.joint2,
            msg.joint3,
            msg.wrist
        ]

        if not all(-120 <= angle <= 120 for angle in angles):
            self.warn("Out of bounds angle detected")
            return
        if not -90 <= msg.joint1 <= 66:
            self.warn("Servo 2 must be between -90 and 66 degrees")
            return
        if not 0 <= msg.gripper < 650:
            self.warn("Gripper pulse must be between 0 and 649")
            return
        targets = [
            (i, atp(angle)) for i, angle in enumerate(angles, start=1)
        ]
        targets.append((10, msg.gripper))
        duration = 1.0 # Time to complete motion
        self.servo_pub.publish(SetBusServoState(
            duration=duration,
            state=[BusServoState(present_id=[1, servo_id], position=[1, pulse])
                   for servo_id, pulse in targets]))


def atp(angle):
    return round((angle + 120) * 1000 / 240)
def pta(pulse):
    return round(pulse * 240.0 / 1000.0 - 120.0)

def main():
    rclpy.init()
    node = ArmControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    

if __name__=="__main__":
    main()
