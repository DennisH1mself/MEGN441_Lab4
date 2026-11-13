#!/usr/bin/env bash
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
robot=ubuntu@192.168.55.1

rsync -avz ./src "$robot:~/l4b3_ws/"

ssh "$robot" 'bash -se' <<'REMOTE'
source /opt/ros/humble/setup.bash
source ~/third_party/orbbec_ws/install/local_setup.bash
cd ~/l4b3_ws
colcon build --symlink-install \
  --packages-select rosbot_msgs rosbot_arm block_detector nav_manager rosbot rosbot_nav \
  --packages-ignore orbbec_camera orbbec_camera_msgs orbbec_description
REMOTE
