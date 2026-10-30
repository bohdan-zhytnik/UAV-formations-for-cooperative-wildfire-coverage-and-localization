#!/usr/bin/env bash
set -euo pipefail
# First take off successfully. World and stable_origin coincide approximately
# at the fixed (0,0) spawn; stamped world_origin avoids local-frame ambiguity.
ros2 service call /uav1/control_manager/reference mrs_msgs/srv/ReferenceStampedSrv   "{header: {frame_id: 'uav1/world_origin'}, reference: {position: {x: 8.0, y: 0.0, z: 5.0}, heading: 0.0}}"
