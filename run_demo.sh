#!/usr/bin/env bash
# Launch the full ASTRA demo: Gazebo + Nav2 + the LLM agent.
#   ./run_demo.sh [nav2_delay] [agent_delay]
# Raise the delays on a slow machine rather than debugging phantom
# "Failed to bring up all requested nodes" errors. Defaults 45 / 85.
set -e
cd "$(dirname "$0")"
ROOT="$PWD"

[ -d install ] || { echo "Not built yet - run ./setup.sh first." >&2; exit 1; }

NAV2_DELAY="${1:-45.0}"
AGENT_DELAY="${2:-85.0}"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

PYV=$(./.venv/bin/python -c 'import sys;print(f"python{sys.version_info.major}.{sys.version_info.minor}")')
export PYTHONPATH="$PYTHONPATH:$ROOT/.venv/lib/$PYV/site-packages"

# The agent also reads config/openai.env, but exporting means no rebuild is
# needed if someone swaps the key.
if [ -z "${MISTRAL_API_KEY:-}" ] && [ -f src/astra_llm_agent/config/openai.env ]; then
  K=$(grep -E '^[[:space:]]*MISTRAL_API_KEY[[:space:]]*=' \
        src/astra_llm_agent/config/openai.env | head -1 | cut -d= -f2- | tr -d " \"'")
  [ -n "$K" ] && export MISTRAL_API_KEY="$K"
fi

echo "Launching (nav2_delay=$NAV2_DELAY agent_delay=$AGENT_DELAY)."
echo "Wait for BOTH 'Managed nodes are active' lines before prompting."
echo

# mistral-small, not -large: the free tier 429s on the large model after ~2 calls.
exec ros2 launch astra_llm_agent astra_full_demo.launch.py \
     model:=mistral-small-latest \
     nav2_delay:="$NAV2_DELAY" agent_delay:="$AGENT_DELAY"
