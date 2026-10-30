#!/usr/bin/env bash
set -euo pipefail
if [[ -z "${APPTAINER_NAME:-}" ]]; then
  echo 'Start this from inside ./wrapper.sh.' >&2; exit 1
fi
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ../scripts/env.sh
python3 ../scripts/prepare.py
if tmux -L mrs_fire has-session -t fire_demo 2>/dev/null; then
  echo 'fire_demo already exists. Attach with: tmux -L mrs_fire attach -t fire_demo'
else
  if pgrep -f '^gz sim server' >/dev/null; then
    echo 'Another Gazebo server is running. Stop the old simulation first.' >&2; exit 1
  fi
  tmuxinator start -p ./session.yml
fi
if [[ "${1:-}" != '--no-attach' ]]; then
  if [[ -n "${TMUX:-}" ]]; then
    echo 'Created fire_demo. Detach from the current tmux, then: tmux -L mrs_fire attach -t fire_demo'
  else
    exec tmux -L mrs_fire attach -t fire_demo
  fi
fi
