#!/usr/bin/env bash
# Interactive console for talking to the running ASTRA agent.
# Run this in a SECOND terminal, after ./run_demo.sh is up.
set -e
cd "$(dirname "$0")"

[ -d install ] || { echo "Not built yet - run ./setup.sh first." >&2; exit 1; }

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

cat <<'EOF'
Try:
  Go to the medical bay
  go fix the engine
  Where can you go?
  Where is the robot?
EOF
exec ros2 run astra_llm_agent prompt_cli
