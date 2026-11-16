import rclpy
from rclpy.node import Node
from rosbot_msgs.msg import IKTarget, JointControl
from rosbot_msgs.action import Pick, Place

#Action server bullshit import
import asyncio
import threading
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np


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


def inverse_kinematics(x, y, z, roll, pitch):
    if not np.all(np.isfinite([x, y, z, roll, pitch])):
        raise ValueError('IK target must contain finite numbers.')
    radius = np.hypot(x, y)
    if radius < 1e-9:
        raise ValueError('Target is on the base axis; rotation is undefined.')
    rotation = np.rad2deg(np.arctan2(y, x))
    # Wrist center relative to the shoulder; preserve the radial sign.
    v3z = z - l0 - (l3 + tool_link) * np.sin(np.deg2rad(pitch))
    v3rxy = radius - (l3 + tool_link) * np.cos(np.deg2rad(pitch))
    distance = np.hypot(v3rxy, v3z)
    if distance < 1e-9:
        raise ValueError('Wrist center is at the shoulder; IK is singular.')
    if distance > l1 + l2 + 1e-9 or distance < abs(l1 - l2) - 1e-9:
        raise ValueError('Target is outside the arm reach.')

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


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        #self.subscriber = self.create_subscription(
        #    IKTarget, '/ik_target', self.ik_callback, 10
        #)
        self.publisher = self.create_publisher(
            JointControl, '/joint_control', 10
        )

        self.open_grip = 100
        self.close_grip = 500
        self.last_angles = None
        self.cur_grip = self.open_grip
        
        #Supposidly makes it so only one action can occur at a time
        self._lock = threading.Lock()
        self._busy = False
        group = ReentrantCallbackGroup()

        #Form for action server from what the internet tells me 

        #ActionServer(
        #self refers to the current instance of ik_node (this in c++) | Pick and '/pick' or Place and '/place' specifies the desired action
        #execute_callback specifies the function that executes the action once a goal is accepted
        #gosl_callback specifies that function to decide whether or not to accept a new action request
        #cancel_callback specifies a function that decides if the client is allowed to cancel an action request
        #callback_group assigns the server to a callback group that controls how its callbacks may execute concurrently
        #)


        #defining pick server
        self.pick_server = ActionServer(
                self, Pick, '/pick',
                execute_callback=self.pick_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=group
                )
        #defining place server
        self.place_server = ActionServer(
                self, Place, '/place',
                execute_callback=self.place_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=group
                )




    #def ik_callback(self, msg):
    #    try:
    #        if not 0 <= msg.gripper < 650:
    #            raise ValueError('Gripper pulse must be between 0 and 649.')
    #        angles = self.inverse_kinematics(
    #            msg.x, msg.y, msg.z, msg.roll, msg.pitch
    #        )
    #    except ValueError as error:
    #        self.get_logger().warn(str(error))
    #        return

    #    response = JointControl()
    #    response.rotation, response.joint1, response.joint2, response.joint3, response.wrist = [
    #        int(round(angle)) for angle in angles
    #    ]
    #    response.gripper = msg.gripper
    #    self.publisher.publish(response)

    def inverse_kinematics(self, x, y, z, roll, pitch):
        return inverse_kinematics(x, y, z, roll, pitch)

    def goal_callback(self, request):

        with self._lock:
            if self._busy:
                return GoalResponse.REJECT

            self._busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def pick_callback(self, goal_handle):
        return await self.run_action(goal_handle, Pick, True)

    async def place_callback(self,goal_handle):
        return await self.run_action(goal_handle,Place,False)

    async def run_action(self,goal_handle,action_type,picking):
        #Still need to write implementation, Chat's gonna help with this guy
        pass
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

    def send_grip(self,gripper):
        if self.last_angles is None:
            raise ValueError('idk some bs')

        self.write_joints(self.last_angles, gripper)

    

def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
