#!/usr/bin/env python3
"""
generate_world.py - builds spacecraft_clean.sdf AND spacecraft_map.pgm/.yaml
from ONE geometry definition, then proves every room is reachable.

Why this exists: the original hand-written world had 18m corridor walls that
ran straight through the rooms and sealed every doorway. The saved map was
also captured from a different, much smaller area. World and map disagreed,
and no amount of Nav2 tuning could fix a world the robot physically could
not traverse. Generating both from one list of walls makes that class of bug
impossible.

Run from the package root:
    python3 tools/generate_world.py
"""

import os
import numpy as np
from PIL import Image
from collections import deque
from scipy import ndimage

# ---------------------------------------------------------------- geometry --
WALL_T = 0.15      # wall thickness
WALL_H = 2.5       # wall height
DOOR_W = 1.6       # doorway width. With robot_radius 0.2 and inflation 0.35
                   # this leaves a ~0.9m uninflated centre band - enough that the
                   # planner drives through cleanly instead of hesitating.
ROOM = 5.0         # room interior size (square)

# Room centres. Top row and bottom row, three each.
BOTTOM_Y = 0.0
TOP_Y = 6.5
ROOM_X = [-7.0, 0.0, 7.0]

# Corridor strip runs between the two rows: y from 2.5 to 4.0 (1.5m wide)
BOTTOM_ROOM_TOP = BOTTOM_Y + ROOM / 2      # 2.5
TOP_ROOM_BOTTOM = TOP_Y - ROOM / 2         # 4.0

ROOMS = {
    'cargo_bay':         (ROOM_X[0], BOTTOM_Y, 'top',    (0.8, 0.8, 0.2)),
    'medical_bay':       (ROOM_X[1], BOTTOM_Y, 'top',    (0.2, 0.8, 0.8)),
    'engine_room':       (ROOM_X[2], BOTTOM_Y, 'top',    (0.8, 0.4, 0.2)),
    'command_module':    (ROOM_X[0], TOP_Y,    'bottom', (0.2, 0.4, 0.8)),
    'sleeping_quarters': (ROOM_X[1], TOP_Y,    'bottom', (0.8, 0.2, 0.2)),
    'airlock':           (ROOM_X[2], TOP_Y,    'bottom', (0.2, 0.8, 0.2)),
}

# Static obstacles: (name, x, y, kind, dims, rgb)
#   box      -> dims = (sx, sy, sz)
#   cylinder -> dims = (radius, length)
# All deliberately placed OFF the room centre so goal poses stay collision-free.
OBSTACLES = [
    ('crate1', -8.3,  1.3, 'box', (0.6, 0.6, 0.5), (0.4, 0.25, 0.1)),
    ('crate2', -5.9, -1.3, 'box', (0.6, 0.6, 0.5), (0.4, 0.25, 0.1)),
    ('crate3', -8.1, -1.6, 'box', (0.6, 0.6, 0.5), (0.4, 0.25, 0.1)),
    ('bed1',   -1.5,  1.2, 'box', (1.0, 0.5, 0.4), (0.9, 0.9, 0.9)),
    ('bed2',    1.5, -1.2, 'box', (1.0, 0.5, 0.4), (0.9, 0.9, 0.9)),
    ('engine_core', 8.3, -1.2, 'cylinder', (0.6, 0.9), (0.2, 0.2, 0.3)),
    ('supply_rack', 0.0,  7.8, 'box', (1.2, 0.4, 0.6), (0.5, 0.5, 0.55)),
]

# Goal poses used by locations.yaml (room centres unless an obstacle is there)
GOALS = {
    'command_module':    (-7.0, 6.5),
    'sleeping_quarters': (0.0, 6.2),
    'airlock':           (7.0, 6.5),
    'cargo_bay':         (-7.0, 0.0),
    'medical_bay':       (0.0, 0.0),
    'engine_room':       (7.0, 0.3),
}

OUTER = dict(xmin=-10.0, xmax=10.0, ymin=-3.0, ymax=9.5)


def h_wall_with_door(x0, x1, y, door_x=None):
    """Horizontal wall from x0..x1 at height y, optional doorway centred on door_x."""
    out = []
    if door_x is None:
        out.append(((x0 + x1) / 2, y, abs(x1 - x0), WALL_T))
        return out
    left_end = door_x - DOOR_W / 2
    right_start = door_x + DOOR_W / 2
    if left_end > x0:
        out.append(((x0 + left_end) / 2, y, left_end - x0, WALL_T))
    if x1 > right_start:
        out.append(((right_start + x1) / 2, y, x1 - right_start, WALL_T))
    return out


def v_wall(x, y0, y1):
    return [(x, (y0 + y1) / 2, WALL_T, abs(y1 - y0))]


def build_walls():
    """Returns list of (cx, cy, sx, sy) axis-aligned wall footprints."""
    walls = []
    half = ROOM / 2

    for name, (cx, cy, door_side, _rgb) in ROOMS.items():
        x0, x1 = cx - half, cx + half
        y0, y1 = cy - half, cy + half
        # side walls always solid
        walls += v_wall(x0, y0, y1)
        walls += v_wall(x1, y0, y1)
        if door_side == 'top':
            walls += h_wall_with_door(x0, x1, y0)            # solid bottom
            walls += h_wall_with_door(x0, x1, y1, door_x=cx)  # door into corridor
        else:
            walls += h_wall_with_door(x0, x1, y1)            # solid top
            walls += h_wall_with_door(x0, x1, y0, door_x=cx)  # door into corridor

    # Outer boundary - closes the corridor ends and the alcoves between rooms
    o = OUTER
    walls += h_wall_with_door(o['xmin'], o['xmax'], o['ymin'])
    walls += h_wall_with_door(o['xmin'], o['xmax'], o['ymax'])
    walls += v_wall(o['xmin'], o['ymin'], o['ymax'])
    walls += v_wall(o['xmax'], o['ymin'], o['ymax'])
    return walls


# ------------------------------------------------------------------- SDF ----
def sdf_box(name, x, y, z, sx, sy, sz, rgb, static=True):
    r, g, b = rgb
    return f"""    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
      </link>
    </model>
"""


def sdf_cylinder(name, x, y, z, radius, length, rgb):
    r, g, b = rgb
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>
      </link>
    </model>
"""


def generate_sdf(walls, path):
    parts = ["""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="spacecraft_clean">

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <!-- Sensors system MUST be a world plugin - it owns the render pass that
         produces gpu_lidar data. Without it /scan is silent and Nav2 is blind. -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <direction>0 -0.3 -1</direction>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.6 0.6 0.6 1</specular>
      <cast_shadows>false</cast_shadows>
    </light>

    <light name="overhead" type="point">
      <pose>0 3 9 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <attenuation>
        <range>40</range><constant>0.3</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- Opens the Gazebo camera looking down at the whole station instead of
         at the origin, so the demo video framing is right immediately. -->
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <property key="showTitleBar" type="bool">false</property>
          <property key="state" type="string">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light>
        <background_color>0.05 0.05 0.10</background_color>
        <camera_pose>0 -14 22 0 0.95 1.5708</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager"/>
      <plugin filename="InteractiveViewControl" name="Interactive view control"/>
      <plugin filename="CameraTracking" name="Camera Tracking"/>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">floating</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">floating</property>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time_factor>true</real_time_factor>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree"/>
    </gui>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>0.45 0.45 0.48 1</ambient><diffuse>0.5 0.5 0.53 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
      </link>
    </model>

"""]

    # Coloured floor tiles per room (visual only - thin, below LiDAR height)
    parts.append("    <!-- Room floors (visual identity for the demo video) -->\n")
    for name, (cx, cy, _d, rgb) in ROOMS.items():
        parts.append(sdf_box(f'floor_{name}', cx, cy, 0.01,
                             ROOM, ROOM, 0.02, rgb))

    parts.append("\n    <!-- Walls -->\n")
    for i, (cx, cy, sx, sy) in enumerate(walls):
        parts.append(sdf_box(f'wall_{i}', cx, cy, WALL_H / 2,
                             sx, sy, WALL_H, (0.72, 0.72, 0.75)))

    parts.append("\n    <!-- Obstacles -->\n")
    for (name, x, y, kind, dims, rgb) in OBSTACLES:
        if kind == 'box':
            sx, sy, sz = dims
            parts.append(sdf_box(name, x, y, sz / 2, sx, sy, sz, rgb))
        else:
            radius, length = dims
            parts.append(sdf_cylinder(name, x, y, length / 2, radius, length, rgb))

    parts.append("  </world>\n</sdf>\n")

    with open(path, 'w') as f:
        f.write(''.join(parts))
    return path


# ------------------------------------------------------------------- MAP ----
RES = 0.05
LIDAR_Z = 0.30   # must match the URDF's lidar_joint origin z


def generate_map(walls, pgm_path, yaml_path, pad=0.6):
    o = OUTER
    xmin, xmax = o['xmin'] - pad, o['xmax'] + pad
    ymin, ymax = o['ymin'] - pad, o['ymax'] + pad
    w = int(round((xmax - xmin) / RES))
    h = int(round((ymax - ymin) / RES))
    grid = np.full((h, w), 255, np.uint8)   # 255 = free

    def stamp(cx, cy, sx, sy):
        px0 = int((cx - sx / 2 - xmin) / RES)
        px1 = int((cx + sx / 2 - xmin) / RES)
        py0 = int((cy - sy / 2 - ymin) / RES)
        py1 = int((cy + sy / 2 - ymin) / RES)
        px0, px1 = max(0, px0), min(w, px1)
        py0, py1 = max(0, py0), min(h, py1)
        grid[py0:py1, px0:px1] = 0

    for (cx, cy, sx, sy) in walls:
        stamp(cx, cy, sx, sy)

    # Only obstacles tall enough to be seen by the LiDAR belong on the map
    for (name, x, y, kind, dims, rgb) in OBSTACLES:
        if kind == 'box':
            sx, sy, sz = dims
            if sz >= LIDAR_Z:
                stamp(x, y, sx, sy)
        else:
            radius, length = dims
            if length >= LIDAR_Z:
                stamp(x, y, radius * 2, radius * 2)

    Image.fromarray(np.flipud(grid), mode='L').save(pgm_path)
    with open(yaml_path, 'w') as f:
        f.write(f"""image: spacecraft_map.pgm
mode: trinary
resolution: {RES}
origin: [{xmin}, {ymin}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
    return grid, (xmin, ymin, w, h)


# --------------------------------------------------------------- VERIFY ----
def verify(grid, meta, robot_radius=0.22):
    xmin, ymin, w, h = meta
    free = (grid == 255)
    r_px = max(1, int(round(robot_radius / RES)))
    nav = ndimage.binary_erosion(free, np.ones((2 * r_px + 1,) * 2, bool))

    def cell(x, y):
        return (int((y - ymin) / RES), int((x - xmin) / RES))

    start = cell(*GOALS['command_module'])
    if not nav[start]:
        print('  !! start pose itself is not navigable')
        return False

    seen = np.zeros_like(nav)
    q = deque([start])
    seen[start] = True
    while q:
        cy, cx = q.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < h and 0 <= nx < w and nav[ny, nx] and not seen[ny, nx]:
                seen[ny, nx] = True
                q.append((ny, nx))

    ok = True
    print(f'  Connectivity with {robot_radius}m inflation, from command_module:')
    for name, (x, y) in GOALS.items():
        c = cell(x, y)
        good = bool(nav[c]) and bool(seen[c])
        ok &= good
        status = 'REACHABLE' if good else ('BLOCKED' if not nav[c] else 'NO PATH')
        print(f'    {name:20s} ({x:5.1f},{y:5.1f})  {status}')
    return ok


if __name__ == '__main__':
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    walls = build_walls()
    print(f'Built {len(walls)} wall segments')

    sdf_path = os.path.join(here, 'worlds', 'spacecraft_clean.sdf')
    generate_sdf(walls, sdf_path)
    print(f'Wrote {sdf_path}')

    pgm = os.path.join(here, 'maps', 'spacecraft_map.pgm')
    ymlp = os.path.join(here, 'maps', 'spacecraft_map.yaml')
    grid, meta = generate_map(walls, pgm, ymlp)
    print(f'Wrote {pgm} ({meta[2]}x{meta[3]}px)')

    print()
    if verify(grid, meta):
        print('\n  ALL SIX ROOMS REACHABLE.')
    else:
        print('\n  *** CONNECTIVITY FAILURE - fix geometry before shipping ***')
        raise SystemExit(1)
