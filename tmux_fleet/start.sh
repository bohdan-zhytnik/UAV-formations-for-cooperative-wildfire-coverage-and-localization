#!/usr/bin/env bash
set -euo pipefail
if [[ -z "${APPTAINER_NAME:-}" ]]; then
  echo 'Run this from inside the MRS Apptainer.' >&2; exit 1
fi
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ../scripts/fleet_env.sh
python3 ../scripts/prepare_fleet.py
if tmux -L mrs_fire_fleet has-session -t fire_fleet 2>/dev/null; then
  echo 'Fleet already running.'
else
  if pgrep -f '^gz sim server|[/]rmw_zenohd' >/dev/null; then
    echo 'Stop the previous simulation/router first; this fleet uses the normal ROS domain.' >&2; exit 1
  fi
  tmuxinator start -p ./session.yml
fi
if [[ "${1:-}" != '--no-attach' ]]; then
  if [[ -n "${TMUX:-}" ]]; then
    echo 'Detach, then: tmux -L mrs_fire_fleet attach -t fire_fleet'
  else
    exec tmux -L mrs_fire_fleet attach -t fire_fleet
  fi
fi
