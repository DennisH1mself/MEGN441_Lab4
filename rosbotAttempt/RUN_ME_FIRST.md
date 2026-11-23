# ROSBot Lab 4 — camera-guided pickup and delivery

This workspace uses the **exact parameter values in the supplied screenshot**. It is a source-code implementation, **not a hardware-verified autonomous delivery**. Keep an emergency stop available and clear the arm's workspace before running. The robot will not start a delivery when you launch RViz or launch the sequencer: only `/start_delivery` starts it.

## Your configuration

| Parameter | Value | Frame / unit |
|---|---|---|
| start_pose | `[-3.543, -2.598, 37.640]` | map x/y meters, yaw degrees |
| estimated_block | `[-2.988, -2.252]` | map x/y meters |
| approach_yaw_deg | `37.64` | map degrees |
| block_color | `red` | detected class |
| viewing_distance_m | `0.65` | meters |
| stand_off_m | `0.26` | meters; first refined goal |
| detection_window_m | `0.65` | meters from estimated map block |
| pick_pitch_deg / pick_roll_deg | `-45.0` / `0.0` | degrees |
| grasp_offset | `[0.0, 0.0, 0.0]` | base_footprint meters |
| dropoff_pose | `[-0.779, -0.410, 37.64]` | map x/y meters, yaw degrees |
| place_pose | `[0.25, 0.0, 0.04, -45.0, 0.0]` | base_footprint x/y/z meters, pitch/roll degrees |

**Crucial:** `estimated_block` is only a guess. The sequencer takes a **new camera detection** to compute both the refined navigation goal (map coordinates) and the final grasp goal (base_footprint coordinates). `place_pose` is not a map position. The standalone IK solver accepts both the supplied place pose and stow pose mathematically, but physical clearances and gripping remain unverified.

## State machine in this package

1. Operator supplies start and course parameters; robot is physically at start. The sequencer waits **idle** for a service call.
2. On start, it publishes `/initialpose` to AMCL, requests stationary localization updates when available, and waits for localization plus stopped odometry.
3. It commands `/stow` and verifies arm joint feedback shows stow.
4. It sends `/navigate` toward an **initial viewing pose** calculated by offsetting the estimated block by 0.65 m at the specified yaw.
5. It listens for **red block detection**. Once a current detection is transformed into map coordinates near the estimate, it cancels the *active first navigation action*, waits for cancellation/terminal status, then sends a **camera-refined approach goal** at 0.26 m stand-off.
6. While approaching, if the camera-derived block pose becomes IK-reachable it cancels the refined navigation action. Otherwise it lets the refined goal finish. If Nav2 aborts because it can't plan a valid path, it retries once at 0.30 m and once at 0.32 m, then stops with a specific error; these attempts do not bypass safety checks.
7. After the base is stationary, it obtains a **fresh** red block detection in the robot's `base_footprint` frame. It validates the gripper pose against the IK and the reach envelope, then sends `/pick`.
8. The IK node moves the gripper to the block, closes the gripper, and returns to stow. The sequencer waits for Pick success *and* fresh measured stow joint angles.
9. It sends `/navigate` toward the supplied `dropoff_pose`. At the specified goal tolerance it cancels an active navigation goal; if Nav2 finishes successfully first it uses that result. It waits until the base is stopped and at the intended parking pose.
10. It sends `/place`, waits for opening and stowing, verifies final joint feedback, and reports completion.

**Limitations:** Action success means commanded timed steps completed; it is not proof that the physical gripper secured the block. No collision-free arm trajectory, force/contact sensing, or grasp confirmation is provided. An emergency stop is required for physical trials.

## 1. Clean build and source

Open a terminal on the robot (examples use zsh, matching your screenshot). Extract the ZIP so the top-level `rosbotAttempt` folder contains `src/` and `delivery.yaml`. The robot's underlying base workspace may already be sourced by its shell.

```bash
cd ~/rosbotAttempt
source /opt/ros/humble/setup.zsh
# Only if needed for your robot's base packages:
source ~/ros2_ws/install/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

If you previously used a different copy of `rosbot_msgs`, avoid sourcing several different compiled workspaces on top of each other. Stale Python message bindings can cause `RuntimeError: Unable to convert call argument to Python object` during a ROS subscription. A clean build of the new workspace in a new shell is recommended.

Open each terminal in this workspace and source `install/setup.zsh`. If working in Bash, use the matching `.bash` setup scripts instead.

## 2. Launch components in separate terminals

Terminal 1 — RGB-D camera and lidar:

```bash
ros2 launch rosbot sensors_launch.py
```

Terminal 2 — map + Nav2:

```bash
ros2 launch rosbot_nav bringup_launch.py \
  map:="$PWD/src/rosbot_nav/maps/Robo_Lab.yaml" \
  params_file:="$PWD/src/rosbot_nav/params/nav2_params.yaml" \
  use_sim_time:=false
```

Terminal 3 — RViz, if needed for initial pose and inspection:

```bash
ros2 launch rosbot_nav rviz_launch.py
```

Terminal 4 — arm controller and arm actions:

```bash
ros2 launch rosbot_arm arm_launch.py
```

Terminal 5 — color/depth block detector:

```bash
ros2 run block_detector block_detector
```

Terminal 6 — navigation relay to Nav2:

```bash
ros2 run nav_manager nav_manager
```

**Do not click the `Nav2 Goal` RViz button** during setup; that orders Nav2 to drive immediately. `2D Pose Estimate` only sets localization, and `Publish Point` publishes a map point without driving. Cancel any leftover Nav2 task before starting.

## 3. Check topics, action servers, and localization before a run

```bash
ros2 action list -t
# Expect /navigate (NavigateToPose), /navigate_to_pose, /stow, /pick, /place
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /odom --once
ros2 topic echo /controller_manager/joint_states --once
ros2 topic echo /block_detector/block_detections --once
```

The last echo only produces output when a colored block is detected. It must print `color: red`, finite position values, and a valid camera `header.frame_id` when the block is visible. Confirm the camera's TF chain to `base_footprint` before running. If `/amcl_pose` is silent, localize with `2D Pose Estimate` at the **actual current robot position**, and confirm laser/map alignment. The sequencer also attempts `/request_nomotion_update` in localization.

Before enabling the robot motors, check that the desired estimated viewing and refined approach points are in navigable free space, not inside an inflated obstacle. The exact initial viewing pose from your parameters is approximately `(-3.503, -2.649, 37.64°)`, only ~6.5 cm from your supplied start; the refined goal will be camera-dependent.

## 4. Launch sequencer — it starts IDLE

```bash
cd ~/rosbotAttempt
source install/setup.zsh
ros2 run delivery_sequencer sequencer --ros-args \
  --params-file "$PWD/delivery.yaml"
```

You should see **Idle**. This node will not command motion merely because it started. Check that the robot is stationary at the start, that the grasp area is clear, and someone can use the physical emergency stop.

In a separate sourced terminal, start **one** mission:

```bash
ros2 service call /start_delivery std_srvs/srv/Trigger "{}"
```

The service returns `success=True` when it *accepted the mission*, not when the mission was completed. Watch sequencer logs for state transitions. To cancel:

```bash
ros2 service call /stop_delivery std_srvs/srv/Trigger "{}"
```

For a second run, stop/restart the sequencer process (one mission per process) and return the robot to the start; a second `/start_delivery` on the same process will be rejected. The service requests cancellation but is NOT an instantaneous motor kill; use the physical stop for that.

## 5. If it stops

- `LOCALIZING` timeout: check `/amcl_pose`, `map -> base_footprint`, live `/odom`, `start_pose`, and whether the robot is stationary. New code retries the AMCL initialization request periodically.
- `STOWING` / `CHECKING_INITIAL_STOW`: check `/joint_control` and `/controller_manager/joint_states`, servo calibration, stow joint tolerance and gripper clearance.
- `DRIVING_TO_ESTIMATE` / `APPROACHING_BLOCK` abort: check the `planner_server` and `controller_server` terminals and global costmap. Your earlier abort was a **no valid path** to a refined goal. The code now tries two larger stand-offs, but if they're blocked too it stops; fix the map/obstacle or approach geometry rather than blindly clearing costmaps.
- `FINDING_GRASP`: verify red block detection publishes, camera depth units and TF are correct, `grasp_offset` / pitch are calibrated, and the robot is stopped and within the valid IK workspace.
- `PLACING`: `place_pose` is base-relative. The supplied values pass the mathematical IK test, but the release height must be physically checked.
- `Unable to convert call argument to Python object`: suspect mixed or stale `rosbot_msgs` code generation/bindings. In a fresh shell source only the robot's base and this workspace, clean-build this workspace, then retest `ros2 topic echo` on the relevant topic.

### What was changed in this revision

- Applied all 13 supplied YAML values at both `delivery.yaml` and `src/delivery.yaml` (your working path in screenshots).
- Updated localization to accept AMCL pose **received** after start, and periodically re-send the initial estimate/request a stationary AMCL update.
- Retained the exact detect → cancel estimate → refined navigation → cancel when reachable → fresh detection → Pick → drive → Place sequence.
- Refined approach gets two safe **navigation replanning attempts** after an abort; each still requires reachable camera-derived grasp and a stopped base.
- Action callbacks in IK and navigation relay use reentrant callback groups and multithreaded executors, protecting the arm from accepting two simultaneous goals and helping cancellation callbacks progress.
