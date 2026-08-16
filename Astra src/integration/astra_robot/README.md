# ASTRA — Autonomous Spacecraft Navigation (ROS 2 Jazzy + Gazebo + Nav2)

One package. One robot. One world. One map. Built from scratch on the most
reliable architecture available, with the world and map generated from a
single source of truth and connectivity proven before shipping.

---

## Quickstart

```bash
mkdir -p ~/astra_ws/src
# copy the astra_robot folder into ~/astra_ws/src/
cd ~/astra_ws
rosdep install --from-paths src --ignore-src -r -y   # optional but recommended
colcon build
source install/setup.bash
```

**Terminal 1 — simulation**
```bash
ros2 launch astra_robot spacecraft.launch.py
```
Wait until the astronaut is visible in Command Module (top-left blue room).

**Terminal 2 — navigation**
```bash
ros2 launch astra_robot nav2.launch.py
```
AMCL is pre-seeded at the spawn pose, so you should **not** need to click
"2D Pose Estimate". The laser scan should sit on the map walls immediately.

**Terminal 3 — drive it**
```bash
# By name (this is what the LLM will call):
ros2 run astra_robot astra_navigator --location engine_room

# Or run it as a service and publish names to it:
ros2 run astra_robot astra_navigator
ros2 topic pub --once /astra/go_to std_msgs/msg/String "{data: 'airlock'}"

# Or tour all six rooms (great for the demo video):
ros2 run astra_robot demo_tour
```

**Manual driving** (sanity check that the base works at all):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Plain `Twist` on `/cmd_vel` — no special flags, no stamped-message mismatch.

---

## The six locations

| Key | Coordinates | Role |
|---|---|---|
| `command_module` | (-7.0, 6.5) | Start / "copilot" spot |
| `sleeping_quarters` | (0.0, 6.2) | Where the robot "rests" |
| `airlock` | (7.0, 6.5) | Main demo goal / EVA prep |
| `cargo_bay` | (-7.0, 0.0) | Storage, messiest obstacles |
| `medical_bay` | (0.0, 0.0) | Medical equipment obstacles |
| `engine_room` | (7.0, 0.3) | "Fix the spacecraft" spot |

Defined once in `config/locations.yaml`, with ~17 natural-language aliases
("cockpit", "medbay", "engineering", …) so the LLM doesn't need exact keys.

---

## LLM integration — the only thing left to do

`astra_navigator` is the hook point. Your LLM branch does **not** need to know
about Nav2, action clients, coordinate frames, or quaternions.

**Option A — publish a string (simplest):**
```python
# In your LangChain tool / agent node:
from std_msgs.msg import String
pub = node.create_publisher(String, '/astra/go_to', 10)
pub.publish(String(data='engine_room'))
```

**Option B — import and call directly:**
```python
from astra_robot.astra_navigator import AstraNavigator, resolve_location, LOCATIONS

key = resolve_location("go check the cargo bay")   # -> 'cargo_bay'
if key is None:
    # resolve_location deliberately does NOT guess. Ask the user to clarify —
    # this is exactly the "ask for clarification instead of guessing"
    # behaviour the project workflow doc requires.
    ...
```

Status comes back on `/astra/status` (`std_msgs/String`) as
`ready` / `navigating: X` / `arrived: X` / `task: ...` / `failed: ...`, so the
agent can report progress to the user in natural language.

Arrival "task reactions" (the Engine Room repair message, etc.) live in
`AstraNavigator._task_reaction()` — extend there.

---

## Generating the SLAM map (required by your rubric)

The shipped map is generated geometrically so you can work immediately. Your
rubric's *Mapping & Localization* criterion wants a real `slam_toolbox` map:

```bash
# Terminal 1
ros2 launch astra_robot spacecraft.launch.py
# Terminal 2
ros2 launch astra_robot slam.launch.py
# Terminal 3 — drive slowly through ALL SIX rooms
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Terminal 4 — when the map looks complete in RViz
ros2 run nav2_map_server map_saver_cli -f ~/spacecraft_map
cp ~/spacecraft_map.pgm ~/spacecraft_map.yaml ~/astra_ws/src/astra_robot/maps/
cd ~/astra_ws && colcon build && source install/setup.bash
```

---

## Architecture (for your report's System Architecture section)

```
   LLM agent  ──/astra/go_to──▶  astra_navigator
                                      │  NavigateToPose action
                                      ▼
   RViz2 ◀──────────────────────    Nav2
   (viz, goals)                   ├── AMCL (localisation vs. map)
                                  ├── Planner Server (global path, NavFn)
                                  ├── Controller Server (local, DWB)
                                  └── Costmaps (static + obstacle + inflation)
                                      │  /cmd_vel (geometry_msgs/Twist)
                                      ▼
                                 ros_gz_bridge
                                      │
                                      ▼
                              Gazebo (gz-sim)
                              ├── DiffDrive plugin  → wheels, /odom, odom→base_footprint TF
                              ├── gpu_lidar sensor  → /scan
                              └── spacecraft_clean.sdf world
                                      ▲
                          robot_state_publisher (URDF → TF tree)
```

**TF chain:** `map` →(AMCL)→ `odom` →(DiffDrive)→ `base_footprint`
→(robot_state_publisher, from URDF)→ `base_link` → wheels / `lidar_link`.

---

## Design decisions worth defending in the report

**2D navigation with an astronaut skin.** Real free-flying robots (NASA's
Astrobee) need full 3D motion planning and thruster control. Nav2 is a 2D
ground-navigation stack. We drive a normal differential-drive base and mount
the astronaut mesh as a visual-only layer — believable result, no rebuilding
of the navigation engine. State this plainly as a scope decision.

**No `ros2_control`.** The drivetrain uses Gazebo's built-in DiffDrive plugin,
configured inline in the URDF. `gz_ros2_control` adds a `controller_manager`,
spawners, and an external YAML — three separate places a parameter file can
silently fail to load while the controller still reports "active". We removed
that entire failure class.

**No MoveIt.** The original design had a 2-joint arm with hand-picked
waypoints and explicitly no IK requirement. MoveIt (SRDF, kinematics solvers,
OMPL pipeline) is built for arms needing collision-aware planning and IK — it
would add substantial failure surface for something the rubric never asks for.
The arm was dropped entirely; arrival "task reactions" are status messages,
which satisfies the workflow doc's "colour change / short animation" intent
with nothing extra to break during the demo.

---

## Credit line (required — must appear in report/README)

> This work is based on "Cute Astronaut"
> (https://sketchfab.com/3d-models/cute-astronaut-816bd786fe8b4559a009de6a95582003)
> by haefu (https://sketchfab.com/haefu), licensed under CC-BY-4.0
> (http://creativecommons.org/licenses/by/4.0/).

---

## What I verified vs. what I could not

**Verified programmatically before shipping:**
- URDF/xacro parses; all five YAML files parse; all Python compiles; SDF and
  package.xml are valid XML.
- All six goal poses are inside the map and collision-free at 0.35m clearance.
- **BFS connectivity proof**: every room reachable from every other with 0.22m
  robot inflation applied to the costmap.
- Cross-file consistency: spawn pose == AMCL initial pose == `command_module`;
  bridge `cmd_vel` type matches Nav2's unstamped setting; URDF LiDAR height
  matches the map generator's raycast height.

**Not verified — no ROS 2/Gazebo runtime available to me:**
- It has never been launched. `colcon build` and `ros2 launch` have not run.
- The astronaut mesh scale (`astronaut_scale` in the URDF, currently `1.0`) is
  a guess — the glb's native units are unknown. If it renders enormous or
  invisible, change that one property (try `0.05`) and rebuild.
- Physics tuning (wheel friction, acceleration) is reasonable but untested.

**The bug that broke the original project:** the old world's 18-metre corridor
walls ran straight through all six rooms and sealed every doorway. No amount of
Nav2 tuning could fix a world the robot physically could not traverse. The new
world is generated by `tools/generate_world.py`, which emits the SDF and the
map from the same wall list and refuses to write output unless all six rooms
prove mutually reachable.

---

## Engineering review — issues found and fixed after the first build

A second pass over the whole system caught these. All are fixed in this
version.

| # | Severity | Issue | Fix |
|---|---|---|---|
| 1 | **Critical** | `astra_navigator` called `rclpy.spin_until_future_complete()` from inside a subscription callback while an executor was already spinning the node. The **first LLM command would hang forever**. | Rewrote with `threading.Event` + action done-callbacks; executor always spins on a background thread. |
| 2 | **Critical** | `gz-sim-sensors-system` was declared at *model* scope in the URDF. It is a **world** plugin — at model scope the render pass may never start, leaving `/scan` silent and Nav2 blind. | Moved to the world SDF. |
| 3 | **Critical** | Gazebo topics were relative (`cmd_vel`, `odom`, `tf`, `scan`), so gz scopes them under `/model/astra_robot/...` and the bridge finds nothing. | All topics made absolute (`/cmd_vel`, …). |
| 4 | **High** | `astra_navigator` is run via `ros2 run` with no launch file, so `use_sim_time` stayed **false**. Goal poses were stamped with wall-clock time while Nav2 runs on sim time → TF lookup failures / rejected goals. | Forced `use_sim_time=true` via `parameter_overrides` in the node constructor. |
| 5 | **High** | `inflation_radius: 0.5` in a 1.4 m doorway left only a 0.4 m uninflated band — planner hesitation and oscillation in every doorway. | Doorways widened to 1.6 m; `inflation_radius` → 0.35, `cost_scaling_factor` → 3.0. Re-verified connectivity at full inflation. |
| 6 | Medium | `laser_max_range: 100.0` / `laser_min_range: -1.0` in AMCL did not match the real sensor (0.12–12 m), degrading the likelihood field. | Set to match the URDF exactly. |
| 7 | Medium | `min_vel_x: 0.0` prevented reversing, so Nav2's `backup` recovery behaviour could never execute. | `min_vel_x: -0.15`. |
| 8 | Low | Gazebo could open paused (you hit this earlier) and framed at the origin. | `<gui>` block with `start_paused=false` and a camera posed over the station. |

### Verification performed on the final build

- xacro, all six YAML files, all Python, SDF and package.xml parse cleanly.
- **Connectivity proven by BFS at three inflation levels** — 0.20 m (lethal
  boundary), 0.30 m, and 0.35 m (full `inflation_radius`, worst case). All six
  rooms mutually reachable at every level, so there is a genuinely zero-cost
  path through each doorway rather than a merely legal one.
- Cross-file consistency, all passing: AMCL initial pose == spawn pose ==
  `command_module`; bridge message type matches Nav2's unstamped setting; URDF
  LiDAR height == map raycast height; LiDAR max range == AMCL `laser_max_range`;
  `wheel_separation` == 2x wheel joint offset; wheel joint z == wheel radius
  (wheels touch the floor — the previous build had them floating 0.05 m);
  all gz topics absolute; all `robot_base_frame` == `base_footprint`.


---

## Troubleshooting

| Symptom | Check |
|---|---|
| Robot doesn't move on `/cmd_vel` | `ros2 topic echo /odom` — is `position.x` changing? Is Gazebo *playing*, not paused (▶ bottom-left)? |
| No `/scan` | `ros2 topic list \| grep scan`; the `Sensors` system plugin must be in the world/URDF |
| Nav2 rejects goals | Is the laser scan aligned to the map walls in RViz? If not, re-set 2D Pose Estimate |
| Astronaut huge/invisible | Change `astronaut_scale` in `urdf/astra_robot.urdf.xacro` |
| Changes not taking effect | `colcon build && source install/setup.bash`, then **fully restart Gazebo** — `robot_description` is baked in at spawn time |
| Stale processes | `pkill -f "gz sim"` before relaunching |
