#!/usr/bin/env bash
# Sourced by every tmux pane. ROS setup scripts require nounset to be off.
fire_restore_nounset=false
[[ $- == *u* ]] && fire_restore_nounset=true
set +u
FIRE_PROJECT_PATH=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export FIRE_PROJECT_PATH
export ROS_DISTRO=jazzy
source /opt/ros/jazzy/setup.bash
source "$FIRE_PROJECT_PATH/../../install/setup.bash"
export UAV_NAME=uav1 RUN_TYPE=simulation UAV_TYPE=x500
export RMW_IMPLEMENTATION=rmw_zenoh_cpp USE_SIM_TIME=true
export GZ_PARTITION=mrs_fire_demo
export PLATFORM_CONFIG="$FIRE_PROJECT_PATH/tmux/config/platform.yaml"
export FIRE_SHARE
FIRE_SHARE=$(ros2 pkg prefix --share fire_observation_demo)

if $fire_restore_nounset; then set -u; fi
unset fire_restore_nounset
