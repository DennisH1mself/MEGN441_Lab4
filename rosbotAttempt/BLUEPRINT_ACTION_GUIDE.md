# MEGN441 ROSBot — action-client/server version based on the Waypoint lab

This package retains the existing sensor, arm, IK, navigation, and detector packages.
It refactors two files to follow the same teaching pattern as `ClientNode` and
`ServerNode` in the supplied turtlesim Waypoint example.

## What changed

### `src/nav_manager/nav_manager/nav_manager.py`

- `ActionServer(... '/navigate', execute_callback=self.action_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)` — accepts navigation requests from sequencer.
- `ActionClient(... '/navigate_to_pose')` — forwards requests to Nav2.
- `goal_response_callback()` — checks that Nav2 accepted the goal and registers `result_callback()`.
- `result_callback()` — records Nav2's terminal result.
- `feedback_callback()` — logs how far Nav2 reports remains.
- `action_callback()` — waits without blocking callbacks and forwards cancellation to Nav2.

### `src/delivery_sequencer/delivery_sequencer/sequencer.py`

- `send()` is equivalent to turtlesim's `send_next_goal()`: uses `send_goal_async` with a feedback callback.
- `goal_response_callback()` handles acceptance/rejection and registers result callback.
- `result_callback()` records completion, without starting arbitrary motion inside the callback.
- `update()` is the state machine. Unlike fixed turtle waypoints, it can switch goals on camera detections, requires a stopped base before manipulation, and checks joint feedback before driving.
- `request_cancel()` asks to cancel an active goal; the sequencer **waits for terminal status** before issuing the next navigation goal.

### `src/rosbot_arm/rosbot_arm/ik_node.py`

Unchanged from the latest workspace. Already uses `ActionServer` for `/stow`, `/pick`, and `/place`, with goal, cancel, and execute callbacks. Uses one `JointControl` message for five joints and gripper.

## Configured delivery.yaml

- start_pose: [-3.543, -2.598, 37.640]
- estimated_block: [-2.988, -2.252]
- approach_yaw_deg: 37.64
- block_color: red
- viewing_distance_m: 0.65
- stand_off_m: 0.26 (retries at 0.30 and 0.32 on Nav2 abort)
- dropoff_pose: [-0.779, -0.410, 37.64]
- place_pose: [0.25, 0.0, 0.04, -45.0, 0.0]

Note: `place_pose` is in `base_footprint`, whereas the block estimate, start, and drop-off are map poses. These values are from the user's recent configuration and must be physically verified.

## Launch (Zsh)

Extract `rosbotAttempt` into `~/l4Test2_ws` as desired (keep source under `src`), then in workspace root:

```
source /opt/ros/humble/setup.zsh
# If your robot needs its base overlay, source that before this workspace.
colcon build --symlink-install
source install/setup.zsh
```

In separate terminals (each source the workspace; run in the workspace root so `$PWD` resolves):

1. `ros2 launch rosbot sensors_launch.py`
2. `ros2 launch rosbot_nav bringup_launch.py map:="$PWD/src/rosbot_nav/maps/Robo_Lab.yaml" params_file:="$PWD/src/rosbot_nav/params/nav2_params.yaml" use_sim_time:=false`
3. `ros2 launch rosbot_nav rviz_launch.py`
4. `ros2 launch rosbot_arm arm_launch.py`
5. `ros2 run block_detector block_detector`
6. `ros2 run nav_manager nav_manager`
7. `ros2 run delivery_sequencer sequencer --ros-args --params-file "$PWD/src/delivery.yaml"`

Verify `/pick`, `/place`, `/stow`, `/navigate`, `/navigate_to_pose` with `ros2 action list -t`. In RViz **2D Pose Estimate** is localization, **Nav2 Goal** commands driving. Make sure no old navigation goal is active. The sequencer prints Idle and never starts driving until:

```
ros2 service call /start_delivery std_srvs/srv/Trigger "{}"
```

Request normal cancellation with:

```
ros2 service call /stop_delivery std_srvs/srv/Trigger "{}"
```

Use the robot's physical emergency stop for immediate stopping.

## Mission order

1. LOCALIZING: initial pose to AMCL, waiting for localization confirmation.
2. STOWING and CHECKING_INITIAL_STOW: command the arm stowed and check joint feedback.
3. DRIVING_TO_ESTIMATE: navigate toward the estimated red block.
4. CANCELING_ESTIMATE: cancel active goal after valid color detection; await result.
5. APPROACHING_BLOCK: navigate toward camera-refined position.
6. CANCELING_APPROACH: stop early if the gripper can reach the target, wait for navigation result.
7. STOPPING_AT_BLOCK / FINDING_GRASP: confirm stopped base and fresh camera data.
8. PICKING / CHECKING_PICK_STOW: command Pick and verify stow.
9. DRIVING_TO_DROPOFF / CANCELING_DROPOFF: navigate, cancel near target and verify stopped.
10. PLACING / CHECKING_FINAL_STOW: place and confirm stow.

## Limitations

- Code was checked for Python syntax; a ROS2 build and hardware run were not available here.
- The physical navigation path and gripper clearance must be tested on the robot.
- Pick currently assumes closing the gripper secured the block; there is no force/contact verification.
- Cancel requests wait for Nav2 terminal response; they are not emergency stops.
- If localization remains timed out, check AMCL freshness, `/odom`, map->base_footprint TF, and consistent `use_sim_time` values across nodes.
