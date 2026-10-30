#!/usr/bin/env bash
set -euo pipefail
for fleet_pane in $(tmux -L mrs_fire_fleet list-panes -s -t fire_fleet -F '#{pane_id}' 2>/dev/null); do
  [[ "${TMUX_PANE:-}" == "$fleet_pane" ]] && continue
  tmux -L mrs_fire_fleet send-keys -t "$fleet_pane" C-c
done
for fleet_wait in {1..30}; do
  fleet_commands=$(tmux -L mrs_fire_fleet list-panes -s -t fire_fleet -F '#{pane_current_command}' 2>/dev/null || true)
  if ! [[ "$fleet_commands" =~ (ros2|python|ruby|fire_rviz|fleet_rviz|gz|component_container) ]]; then break; fi
  sleep 0.3
done
tmux -L mrs_fire_fleet kill-session -t fire_fleet
