# ASTRA — working project

Aboud Elzubair · aabdal35@my.centennialcollege.ca

The spacecraft robot plus the LLM natural-language navigation layer. This is the
code that actually runs — verified end to end in simulation.

---

## Quick start (three commands)

Everything runs in **WSL / Ubuntu 24.04**, not Windows.

```bash
unzip ASTRA.zip && cd ASTRA
./setup.sh          # one time: python deps + colcon build   (~3 min)
./run_demo.sh       # Gazebo + Nav2 + the LLM agent
```

Then in a **second** terminal:

```bash
cd ASTRA && ./prompt.sh
> go fix the engine
```

The extracted folder **is** the colcon workspace — no copying into `~/astra_ws`.

---

## What you need first

- **ROS 2 Jazzy** — `source /opt/ros/jazzy/setup.bash` must work
- **Gazebo Harmonic** (gz-sim 8), **Nav2**, **ros_gz_sim**, **python3-venv**

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                 ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge python3-venv
```

`setup.sh` checks for these and stops with a clear message if any are missing.
It creates its own virtualenv at `.venv/` inside the project and installs
LangChain there — it does not touch your system Python.

---

## ⚠️ There is a live API key in here

`src/astra_llm_agent/config/openai.env` contains a **real Mistral API key**, so
the demo runs with no setup. **Do not commit it** — the team repo has GitHub
secret scanning on, so the push will be rejected and the key burned. It is
already in `.gitignore`; keep it out of `git add`.

---

## What is in here

```
src/
├── astra_robot/       robot URDF, Gazebo world, occupancy map, Nav2 bringup,
│                      RViz config, and the world/map generator
└── astra_llm_agent/   Mistral + LangChain agent: /prompt -> NavigateToPose
docs/                  astronaut visual fix: notes + diff
setup.sh  run_demo.sh  prompt.sh
```

**Where this came from:** `astra_llm_agent` is from the team repo (branch `LLM`).
**`astra_robot` is not in the repo at all** — it only ever existed in the local
workspace, which is why a plain checkout cannot run the simulation and why this
zip exists.

The old `rosbot_description`, `rosbot_nav2_bringup`, `nav2_client` and
`nav2_bt_client` packages are **deliberately not included**. `astra_robot`
replaces all four. Building them alongside it gives conflicting launch files and
the old 97×73 px map (~4.9 × 3.7 m) that every room goal falls outside of. They
are still in the repo if anyone wants them.

---

## Running the pieces separately

```bash
source install/setup.bash
ros2 launch astra_robot spacecraft.launch.py     # Gazebo + robot + bridge
ros2 launch astra_robot nav2.launch.py           # Nav2 + AMCL + RViz
ros2 launch astra_llm_agent llm_agent.launch.py  # agent only
```

## Things that will otherwise cost you an hour

- **Always `model:=mistral-small-latest`.** The free tier returns HTTP 429 on
  `mistral-large-latest` after about two calls. `run_demo.sh` sets this already.
- **No 2D Pose Estimate needed** — AMCL is pre-seeded at the command module.
  Wait for both `Managed nodes are active` lines before sending a prompt.
- **If bringup fails** with `Failed to bring up all requested nodes`, raise the
  delays rather than chasing the error:
  `./run_demo.sh 60 110` (nav2_delay, agent_delay). Defaults are 45 / 85.
- **Navigation takes ~2–3 minutes per room.** The sim runs at roughly 28% real
  time under WSL — budget about 13 s of wall clock per metre of path.
- **Restart WSL (`wsl --shutdown`) before a demo.** WSLg wedges after ~10 GUI
  launches: the world loads and gz topics appear, but the sim clock never
  advances and the robot never spawns. A restart clears it completely.

## Tests

```bash
cd src/astra_llm_agent && python3 -m pytest test/ -v      # 51 tests, no ROS, no API
```

---

## Room coordinates are generated, not hand-typed

`astra_llm_agent/config/rooms.yaml` is copied from
`astra_robot/config/locations.yaml`, produced by
`astra_robot/tools/generate_world.py` — the script that builds the Gazebo world
**and** the occupancy map from one geometry definition, then proves by BFS (at
full 0.35 m costmap inflation) that every pose is collision-free and mutually
reachable.

**Do not hand-edit coordinates.** Re-run that script and re-copy. An earlier
`rooms.yaml` was hand-tuned against a world whose corridor walls sealed every
doorway.

---

## Astronaut visual — two things that break easily

See `docs/README_astronaut_fix.md` for the full story. The traps:

- The LiDAR needs **both** `<visibility_flags>1</visibility_flags>` on the
  astronaut visual **and** `<visibility_mask>4294967294</visibility_mask>` on
  the `gpu_lidar` sensor. `gz sdf -p` warns that `visibility_mask` is "not
  defined in SDF" — ignore it; gz-sensors reads it anyway and it is what keeps
  the astronaut out of the ray cast. Remove either and the robot goes blind.
- `meshes/cute_astronaut_texture.png` must stay **beside**
  `cute_astronaut_textured.glb`. The glb references it by bare filename;
  separate them and the astronaut silently renders flat grey.

---

## Known documentation gap

`src/astra_llm_agent/README.md` is **out of date** — it still describes the
removed `rosbot_*` launch files, a demo launch file that no longer exists,
`mistral-large-latest` as the default, `base_link` as the base frame, "42 unit
tests", and a blockers list that has since been fixed. **Use this file instead**
until it is rewritten.

## Verification status

- Unit tests 51/51.
- Four natural-language navigations verified end to end (sleeping quarters,
  engine room ×2, airlock), final position error 3–6 cm.
- Astronaut visual: closest LiDAR return 2.325 m, 0 of 360 beams under 0.5 m,
  resting pitch 0.00006°.

The astronaut fix has **not** been re-run through a full navigation test since
it was made — the collision-monitor result is an inference from scan data. Worth
one full run before the demo.
