# ASTRA — astronaut visual fix for `astra_robot`

Aboud Elzubair · aabdal35@my.centennialcollege.ca

This makes the astronaut stand upright and centred on the robot base, in its
original Sketchfab colours, **without** blinding the LiDAR.

Everything here is confined to `astra_robot`'s astronaut visual. No Nav2, map,
room coordinates, controller, drivetrain or navigation behaviour is touched.

---

## What was wrong

**1. The astronaut was lying on its side, next to the base.**

The sizing/orientation numbers in the xacro were read off the glb's *raw
vertex data*. Gazebo also applies the glb's **node transforms**, so what it
actually draws is a different size and a different way up. Measured as gz-sim
really loads it (via a headless camera render, at scale 1.0):

| | xacro assumed | actual |
|---|---|---|
| height | 13.997 units | **1.7009 m** — it is a life-size model |
| origin | top of the head, body hanging down | **at the feet** |
| orientation | needed a +90° roll | **already Z-up**, facing −Y |

So `rpy="1.5708 0 0"` tipped an already-upright model onto its side, and the
`mesh_drop` raise then lifted it clear of the base. One wrong premise, both
symptoms.

**2. The astronaut was flat grey.**

`meshes/cute_astronaut.glb` is a re-export that **contains no textures at
all** — 0 images, and material `Astronaut` reduced to a flat
`baseColorFactor [0.8, 0.8, 0.8]`. That is exactly the grey everyone was
seeing. `cute_astronaut.glb.orig` still holds the real 2048×2048 atlas with the
NASA meatball, US flag, Swiss cross and orange suit panels.

**3. Orange rendered as blue.**

Textures embedded *inside* a GLB come through gz-common's in-memory Assimp path
with **red and blue swapped**: orange (245,160,60) arrives as light blue
(60,160,245). White and black are unchanged by an R↔B swap, which is why the
suit still looked "white with blue bits". Writing the PNG out beside the mesh
and referencing it by `uri` routes it through gz-common's normal image loader,
which gets the channel order right.

**4. The visor rendered black.**

Material `visier` sets an orange base colour but never sets `metallicFactor`,
and **glTF's default is 1.0** — a perfect chrome mirror. With no IBL or skybox
in the scene, a mirror reflects nothing and comes out black. Pinning it to 0.0
keeps the authored orange as a plain glossy dielectric.

---

## Files in this package

```
astra_robot/
├── urdf/astra_robot.urdf.xacro       ← replaces yours (drop-in)
└── meshes/
    ├── cute_astronaut_textured.glb   ← NEW
    └── cute_astronaut_texture.png    ← NEW — must sit beside the .glb
```

`astra_robot.urdf.xacro.diff` is the unified diff against the version currently
in the repo, if you would rather apply it by hand than overwrite.

**The `.png` is not optional and not decorative.** The `.glb` refers to it by a
bare relative filename, so if the two are ever separated the mesh silently loses
its texture and goes back to grey. `setup.py`'s `data_files_for('meshes')`
globs `meshes/*`, so it installs automatically — **no `setup.py` change needed.**

`cute_astronaut.glb` and `cute_astronaut.glb.orig` are left exactly as they are.
Nothing references `cute_astronaut.glb` any more.

---

## How to apply

```bash
cd ~/astra_ws/src/astra_robot
cp urdf/astra_robot.urdf.xacro urdf/astra_robot.urdf.xacro.bak   # keep a copy

# from the folder where you unzipped this:
cp -r astra_robot/ ~/astra_ws/src/

cd ~/astra_ws
colcon build --symlink-install --packages-select astra_robot
source install/setup.bash
ros2 launch astra_robot spacecraft.launch.py
```

---

## The resulting geometry

| | |
|---|---|
| mesh | `cute_astronaut_textured.glb` |
| scale | `0.264566` — derived, not hand-typed: `astronaut_height 0.45 m ÷ mesh_height 1.7009 m` |
| visual `rpy` | `0 0 1.5708` — yaw only, a quarter turn to face the robot's +X |
| visual `xyz` | `0.010424 0.008016 -0.000609` — centres the mesh's bbox on the robot's axis |
| joint origin | `0 0 0.21` (`${deck_z}`, the top of the base box) |
| soles / helmet top | z = 0.2100 / z = 0.6594 |
| torso footprint | 0.2625 m fore-aft — fits inside the 0.30 m base |
| arm span | 0.4258 m (T-pose, arms overhang the base as expected) |

**To resize, change one number:** `astronaut_height`, in metres, at the top of
the file. Everything else follows from it.

---

## The LiDAR fix — please do not remove either half

The astronaut's legs stand right through the LiDAR plane at z = 0.30. Two
elements keep it out of the ray cast, and **both are required**:

- `<visibility_flags>1</visibility_flags>` on the astronaut visual, via
  `<gazebo reference="astronaut_visual">`
- `<visibility_mask>4294967294</visibility_mask>` (0xFFFFFFFE) on the
  `gpu_lidar` sensor

`gz sdf -p` prints a warning that `visibility_mask` is "not defined in SDF".
**Ignore it** — sdformat does not formally define the element but gz-sensors
reads it anyway, and it is what actually does the discriminating. Remove it and
the astronaut blinds the LiDAR again.

Two more traps worth knowing:

- The URDF→SDF conversion lumps fixed joints, renaming the visual to
  `base_footprint_fixed_joint_lump__astronaut_visual_visual_1`. The
  `<gazebo reference="...">` block is matched by **link name**, so renaming the
  `astronaut_visual` link silently breaks the fix.
- The visual-only link needs its explicit tiny `<inertial>` (1 g). Without one
  the conversion can lump a default mass in at the mount point and the robot
  pitches.

The GUI camera has no mask, so it still draws the astronaut normally — visible
to people, invisible to the sensor.

---

## Verification (simulation only)

Run in Gazebo Harmonic / gz-sim 8.11, ROS 2 Jazzy, robot stationary in the
command module:

| check | result |
|---|---|
| closest LiDAR return | **2.325 m** (was 0.179 m when blinded) |
| beams under 0.5 m | **0 of 360** |
| Nav2 `PolygonStop` (r = 0.20 m, `min_points: 4`) | cannot latch — nothing is inside even 0.5 m |
| resting pitch | **0.00006°** (1×10⁻⁶ rad) |
| astronaut visible in GUI | yes — see screenshots |

**Scope note:** this was verified in simulation only. Nav2 and the LLM agent
were deliberately *not* run as part of this change, so the collision-monitor
result above is an inference from the scan data rather than a direct
observation. A full navigation run is still worth doing before the demo.

---

## One thing to expect

From a steep top-down camera the astronaut *looks* like it is leaning and
floating beside the base. It is not — that is perspective. The helmet sits
0.66 m up, so from a ~50° camera it projects about 0.52 m toward the viewer,
which is further than the whole 0.30 m base is long. This was checked by
rendering the verified-correct model at that same angle and getting the same
apparent lean. From a level or gently angled view it reads correctly.

If you would rather it never looked that way, sinking the astronaut ~6 cm into
the base (`deck_z` 0.21 → 0.15) makes it read as one object from every angle.
That was tried and deliberately not adopted — say so if you want it.
