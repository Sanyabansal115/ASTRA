# astra_llm_agent — LLM Navigation Integration (ASTRA)

Natural-language control of the ASTRA spacecraft robot.
Say *"Go to the medical bay"* and the robot drives there.

This package is the **AI agent / LLM integration** part of the COMP219 group
project. It is a thin high-level layer on top of the team's existing stack: the
LLM only decides *where* to go, **Nav2 still does all path planning, control and
obstacle avoidance**, exactly as the project brief requires.

It follows the Week 12–13 lab pattern (prompts on `/prompt`, LangChain tools,
Mistral AI), extended with a semantic room table so destinations can be named
instead of typed as raw coordinates.

---

## How it fits into the team's system

```
  "Go to the medical bay"
          │  ros2 topic pub /prompt std_msgs/msg/String
          ▼
  ┌───────────────────────────────────────────────┐
  │ astra_llm_agent  (this package)               │
  │   ChatMistralAI + LangChain AgentExecutor     │
  │   tools: navigate_to_room, list_rooms,        │
  │          navigate_to_coordinates,             │
  │          get_robot_pose                       │
  │   config/rooms.yaml  ← semantic → coordinates │
  └───────────────────────┬───────────────────────┘
                          │ NavigateToPose goal (frame: map)
                          ▼
  ┌───────────────────────────────────────────────┐
  │ Nav2  (rosbot_nav2_bringup — teammate)        │
  │   bt_navigator → planner → controller         │
  └───────────────────────┬───────────────────────┘
                          │ /cmd_vel
                          ▼
  ┌───────────────────────────────────────────────┐
  │ ros_gz_bridge → Gazebo (rosbot_description)   │
  └───────────────────────────────────────────────┘
```

**Nothing in the teammates' packages was modified.** This package uses the same
`navigate_to_pose` action server and the same room coordinates as the existing
C++ menu client (`src/nav2_client/src/navigation_client.cpp`), so the LLM and the
manual menu drive the robot to exactly the same places.

---

## Tools given to the LLM

| Tool | Purpose | Example prompt |
|---|---|---|
| `navigate_to_room(room)` | Send the robot to a named module (looked up in `rooms.yaml`) | "Go to the medical bay" |
| `navigate_to_coordinates(x, y)` | Send the robot to raw map coordinates | "Move to x=2.0 and y=1.5" |
| `list_rooms()` | List every destination and its coordinates | "Where can you go?" |
| `get_robot_pose()` | Report the current pose from TF (`map` → `base_link`) | "Where is the robot?" |

Room names are matched loosely, so *"medbay"*, *"the medical bay"*, *"infirmary"*
and *"sick bay"* all resolve to `medical_bay`. If a destination is not
recognised the tool returns the valid list instead of guessing, and the agent
asks the user which one they meant.

---

## Setup

### 1. Dependencies

Same virtualenv as the Week 12–13 labs (LangChain must be pinned pre-1.0 so
`AgentExecutor` exists):

```bash
cd ~/ASTRA                         # the cloned repo IS the colcon workspace
python3 -m venv .venv              # if you don't have one yet
source .venv/bin/activate
pip install --upgrade pip
pip install "langchain<1.0.0" "langchain-mistralai<1.0.0" pyyaml
deactivate
```

### 2. Mistral API key

Ask the team for the shared lab key. The agent looks for it in this order —
**the key is never committed; only the empty `config/openai.env.example` is in git**:

```bash
# 1. environment variable (simplest, no rebuild needed)
export MISTRAL_API_KEY=your_key_here

# 2. a file outside the repository
mkdir -p ~/.astra_llm
echo 'MISTRAL_API_KEY=your_key_here' > ~/.astra_llm/openai.env

# 3. the lab-style file: copy the template and fill it in.
#    config/openai.env is listed in .gitignore, so it stays local.
cd ~/ASTRA/src/astra_llm_agent
cp config/openai.env.example config/openai.env
nano config/openai.env            # set MISTRAL_API_KEY=
#    then rebuild, because the agent reads the installed copy
```

### 3. Build

```bash
cd ~/ASTRA
source /opt/ros/jazzy/setup.bash
# make the venv packages visible to `ros2 launch` (Week 13 lab step):
export PYTHONPATH=$PYTHONPATH:$HOME/ASTRA/.venv/lib/python3.12/site-packages
colcon build --packages-select astra_llm_agent
source install/setup.bash
```

---

## Running

Three terminals. Terminals 1 and 2 are the teammates' existing launch files,
unchanged.

**Terminal 1 — simulation**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rosbot_description gazebo.launch.py
```

**Terminal 2 — Nav2**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rosbot_nav2_bringup nav2.launch.py
```
Set the robot's initial pose in RViz with **2D Pose Estimate** before sending
any goal, otherwise AMCL does not know where the robot is.

**Terminal 3 — the LLM agent**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export MISTRAL_API_KEY=your_key_here
export PYTHONPATH=$PYTHONPATH:$HOME/ASTRA/.venv/lib/python3.12/site-packages
ros2 launch astra_llm_agent llm_agent.launch.py
```
Wait for: `ASTRA LLM navigation agent ready (model=mistral-large-latest).`

**Or start everything at once** (Gazebo + Nav2 + agent, agent delayed 15 s):
```bash
ros2 launch astra_llm_agent astra_llm_demo.launch.py
```

### Sending prompts

```bash
ros2 topic pub --once /prompt std_msgs/msg/String "{data: 'Go to the medical bay'}"
ros2 topic pub --once /prompt std_msgs/msg/String "{data: 'Navigate to the command module'}"
ros2 topic pub --once /prompt std_msgs/msg/String "{data: 'Go to the cargo bay'}"
ros2 topic pub --once /prompt std_msgs/msg/String "{data: 'Where can you go?'}"
ros2 topic pub --once /prompt std_msgs/msg/String "{data: 'Where is the robot?'}"
ros2 topic pub --once /prompt std_msgs/msg/String "{data: 'Move to x=1.0 and y=2.0'}"
```

Or type sentences interactively:

```bash
ros2 run astra_llm_agent prompt_cli
```

The agent's replies are also published on `/llm_response`:

```bash
ros2 topic echo /llm_response
```

Expected output in the agent terminal (good screenshot material for the report):

```
[astra_llm_nav_agent]: Prompt received: Go to the medical bay

> Entering new AgentExecutor chain...
Invoking: `navigate_to_room` with `{'room': 'medical_bay'}`
[astra_llm_nav_agent]: Sending Nav2 goal -> Medical Bay (x=0.00, y=1.50)
[astra_llm_nav_agent]: Nav2 accepted the goal for Medical Bay.
> Finished chain.
[astra_llm_nav_agent]: Result: The robot is on its way to the Medical Bay.
[astra_llm_nav_agent]: Navigating to Medical Bay - 3.42 m remaining.
[astra_llm_nav_agent]: Arrived at Medical Bay.
```

---

## Files

```
astra_llm_agent/
├── astra_llm_agent/
│   ├── llm_nav_agent.py     ROS 2 node: /prompt → Mistral+LangChain → Nav2 goals
│   ├── rooms.py             room table loading + name matching (no ROS imports)
│   └── prompt_cli.py        interactive console for the demo
├── config/
│   ├── rooms.yaml           semantic → coordinate mapping (the only place goals live)
│   └── openai.env.example   API key template (EMPTY); copy to openai.env locally
├── launch/
│   ├── llm_agent.launch.py       agent only (Gazebo + Nav2 already running)
│   └── astra_llm_demo.launch.py  Gazebo + Nav2 + agent in one command
└── test/
    └── test_rooms.py        42 unit tests for the room table (run without ROS)
```

### Node parameters

| Parameter | Default | Notes |
|---|---|---|
| `rooms_file` | `<share>/config/rooms.yaml` | Semantic location table |
| `env_file` | `<share>/config/openai.env` | Last-resort key source (git-ignored local file) |
| `model` | `mistral-large-latest` | Use `mistral-small-latest` if rate-limited |
| `temperature` | `0.0` | Deterministic tool choice |
| `prompt_topic` / `response_topic` | `/prompt` / `/llm_response` | |
| `nav_action` | `navigate_to_pose` | Nav2 action server name |
| `map_frame` / `robot_base_frame` | `map` / `base_link` | Matches `nav2_params.yaml` |

### Tests

```bash
cd ~/ASTRA/src/astra_llm_agent
python3 -m pytest test/ -v
```

---

## Known blockers (other people's parts — documented, not changed)

These do not come from this package, but the LLM demo cannot succeed until the
first two are fixed by the teammates who own them:

1. **The saved map is far too small.** `src/rosbot_nav2_bringup/maps/spacecraft_map.pgm`
   is 97 × 73 px at 0.05 m/px ≈ **4.9 m × 3.7 m**, origin `(-3.43, -1.18)`, while
   the world spans roughly 19 m × 11 m. Every room coordinate falls outside the
   map, so Nav2 will refuse or abort the goals — for the C++ menu client just as
   much as for the LLM. The map needs to be re-recorded with
   `slam_nav2.launch.py` and saved with `ros2 run nav2_map_server map_saver_cli`.
2. **Two goal points sit inside static obstacles** in
   `src/rosbot_description/worlds/spacecraft_clean.sdf`: `crate3` is centred exactly on the cargo bay
   goal `(-7, 0)` and `engine_machine` (0.6 m radius) on the engine room goal
   `(7, 0)`; the medical bay goal `(0, 0)` sits in the 0.5 m gap between the two
   hospital beds. `rooms.yaml` therefore uses `y = 1.5` for those three rooms
   (each original value is kept in a comment). If the world changes, update
   `rooms.yaml` — no code changes needed.
3. **The three top-row modules may be sealed off.** `corridor_top` has a solid
   18 m wall at `y = 4.75` that crosses the command module, sleeping quarters and
   airlock between their doors (`y = 3.5`) and their interiors, and
   `corridor_bottom` has the same issue at `y = -1.25`. Worth driving the robot
   around in Gazebo to confirm — if it holds, only cargo bay, medical bay and
   engine room are actually reachable.
4. **Two nodes publish `map → odom`.** `nav2.launch.py` starts a static
   `map → odom` transform publisher while AMCL publishes the same transform
   (same in `slam_nav2.launch.py` with slam_toolbox). This will fight AMCL and
   should be removed once localisation works.
5. **`map_server`'s `yaml_filename`** in `nav2_params.yaml` is
   `~/spacecraft_ws/src/...` (a path from another machine); `~` is not expanded by the map server. It works
   only because `nav2.launch.py` overrides it with the `map:` argument.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Missing LangChain/Mistral dependency` | Activate the venv, or `export PYTHONPATH=...` before `ros2 launch` |
| `No Mistral API key found` | `export MISTRAL_API_KEY=...` (see Setup step 2) |
| `Nav2 action server 'navigate_to_pose' is not available` | Nav2 isn't running, or is still starting up |
| `Navigation to X failed … outside the map` | Blocker 1 above — the map is too small |
| Robot doesn't move but the model replies happily | The model answered without calling a tool; re-phrase ("Go to the medical bay") or switch model with `model:=mistral-small-latest` |
| `no transform from 'map' to 'base_link'` | Set the initial pose in RViz with **2D Pose Estimate** |
| Rate-limit / 429 errors from Mistral | Free "Experiment" plan limit — wait, or use `mistral-small-latest` |
