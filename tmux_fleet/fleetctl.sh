#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ../scripts/fleet_env.sh
exec python3 ../scripts/fleetctl.py "$@"
