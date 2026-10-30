#!/usr/bin/env bash
# All shells use the same base environment; the fleet has its own transport partition.
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
export GZ_PARTITION=mrs_fire_fleet
# This is a single-notebook demo. Pin Gazebo transport to loopback so Wi-Fi
# address changes do not interrupt the clock, cameras or PX4 simulation.
export GZ_IP=127.0.0.1
export FIRE_FLEET_SCENARIO_FILE="$FIRE_SHARE/config/fleet/scenario.yaml"
