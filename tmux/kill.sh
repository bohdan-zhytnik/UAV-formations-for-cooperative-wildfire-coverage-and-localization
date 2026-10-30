#!/usr/bin/env bash
set -euo pipefail
# SIGINT ROS launch panes first so PX4, Gazebo, and bridges stop cleanly.
for pane in $(tmux -L mrs_fire list-panes -s -t fire_demo -F '#{pane_id}' 2>/dev/null); do
  [[ "${TMUX_PANE:-}" == "$pane" ]] && continue
  tmux -L mrs_fire send-keys -t "$pane" C-c
 done
sleep 3
tmux -L mrs_fire kill-session -t fire_demo
