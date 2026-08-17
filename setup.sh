#!/usr/bin/env bash
# ASTRA one-time setup: python deps + colcon build.
# The folder you extracted IS the workspace - run this from inside it.
set -e
cd "$(dirname "$0")"
ROOT="$PWD"

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$1"; }
die()  { printf '\n\033[1;31mERROR: %s\033[0m\n\n' "$1" >&2; exit 1; }

say "Checking ROS 2 Jazzy"
[ -f /opt/ros/jazzy/setup.bash ] || die \
"ROS 2 Jazzy not found at /opt/ros/jazzy.
 This project needs ROS 2 Jazzy on Ubuntu 24.04, inside WSL."
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
echo "    ROS_DISTRO=$ROS_DISTRO"

say "Checking simulation dependencies"
MISSING=""
for p in ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ros-gz-sim \
         ros-jazzy-ros-gz-bridge; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"
done
command -v gz >/dev/null 2>&1 || MISSING="$MISSING gz(Gazebo-Harmonic)"
if [ -n "$MISSING" ]; then
  die "missing packages:$MISSING

 Install them with:
   sudo apt update && sudo apt install ros-jazzy-navigation2 \\
        ros-jazzy-nav2-bringup ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge"
fi
echo "    all present"

say "Setting up the Python virtualenv (.venv)"
# LangChain is pinned pre-1.0 because AgentExecutor was removed in 1.0.
if [ ! -d .venv ]; then
  python3 -m venv .venv || die \
"could not create a virtualenv - install it with: sudo apt install python3-venv"
fi
# Clear PYTHONPATH for the pip calls: sourcing ROS puts its site-packages on
# it, and pip's resolver then reports confusing "launch-ros requires
# setuptools, which is not installed" conflicts about packages it does not own.
env PYTHONPATH= ./.venv/bin/pip install --quiet --upgrade pip setuptools
env PYTHONPATH= ./.venv/bin/pip install --quiet \
    "langchain<1.0.0" "langchain-mistralai<1.0.0" pyyaml
PYV=$(./.venv/bin/python -c 'import sys;print(f"python{sys.version_info.major}.{sys.version_info.minor}")')
echo "    langchain installed for $PYV"

say "Building (colcon)"
export PYTHONPATH="$PYTHONPATH:$ROOT/.venv/lib/$PYV/site-packages"
colcon build --symlink-install --packages-select astra_robot astra_llm_agent

say "Checking the API key"
if grep -qE '^[[:space:]]*MISTRAL_API_KEY[[:space:]]*=[[:space:]]*[^[:space:]]' \
     src/astra_llm_agent/config/openai.env 2>/dev/null; then
  echo "    key found in src/astra_llm_agent/config/openai.env"
  echo "    (do NOT commit that file - the repo has secret scanning enabled)"
else
  echo "    no key found. Before running, either:"
  echo "      export MISTRAL_API_KEY=your_key_here"
  echo "      or put it in src/astra_llm_agent/config/openai.env"
fi

say "Done"
cat <<'EOF'
    Next:
      ./run_demo.sh          Gazebo + Nav2 + the LLM agent
      ./prompt.sh            (second terminal) type instructions

    If Gazebo loads but the robot never spawns and the clock never advances,
    WSLg has wedged - run `wsl --shutdown` from Windows and try again.
EOF
