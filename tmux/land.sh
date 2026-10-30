#!/usr/bin/env bash
set -euo pipefail
ros2 service call /uav1/uav_manager/land std_srvs/srv/Trigger '{}'
