"""Unit tests for the semantic room table.

These run without ROS 2 (`pytest test/`), so the name-matching that decides
where the robot drives can be checked on any machine:

    cd src/astra_llm_agent && python -m pytest test/ -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from astra_llm_agent.rooms import (  # noqa: E402
    RoomTable,
    normalise,
    quaternion_to_yaw,
    yaw_to_quaternion,
)

ROOMS_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'rooms.yaml'
)


@pytest.fixture(scope='module')
def table():
    return RoomTable.from_file(ROOMS_FILE)


# ----------------------------------------------------------------- room table
def test_all_six_modules_are_present(table):
    expected = {
        'command_module', 'sleeping_quarters', 'airlock',
        'cargo_bay', 'medical_bay', 'engine_room',
    }
    assert set(table.keys()) == expected


# The generated goal poses, from astra_robot/config/locations.yaml. That file is
# emitted by astra_robot/tools/generate_world.py, which builds the world and the
# occupancy map from one geometry definition and proves by BFS that every pose is
# collision-free and mutually reachable. These are duplicated here (rather than
# only cross-checked below) so the check still runs when astra_robot is absent.
GENERATED_POSES = {
    'command_module': (-7.0, 6.5),
    'sleeping_quarters': (0.0, 6.2),
    'airlock': (7.0, 6.5),
    'cargo_bay': (-7.0, 0.0),
    'medical_bay': (0.0, 0.0),
    'engine_room': (7.0, 0.3),
}


def test_coordinates_match_the_generated_locations(table):
    for key, (x, y) in GENERATED_POSES.items():
        room = table.get(key)
        assert (room.x, room.y) == (x, y), (
            f'{key} has drifted from astra_robot/config/locations.yaml. '
            'Re-run generate_world.py and re-copy; do not hand-edit.'
        )


def test_rooms_yaml_agrees_with_astra_robot_locations(table):
    """Cross-check against the real astra_robot file when it is in the workspace.

    Skips when astra_llm_agent is checked out on its own, which is why
    GENERATED_POSES above is also asserted directly.
    """
    locations_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'astra_robot', 'config', 'locations.yaml',
    )
    if not os.path.exists(locations_file):
        pytest.skip('astra_robot is not in this workspace')

    import yaml
    with open(locations_file, 'r', encoding='utf-8') as handle:
        locations = yaml.safe_load(handle)['locations']

    assert set(locations) == set(table.keys()), (
        'rooms.yaml and locations.yaml disagree on which modules exist'
    )
    for key, value in locations.items():
        room = table.get(key)
        assert (room.x, room.y, room.yaw) == (value[0], value[1], value[2]), (
            f'{key}: rooms.yaml has ({room.x}, {room.y}, {room.yaw}) but '
            f'locations.yaml has {tuple(value)}'
        )


# The saved map is 424 x 274 px at 0.05 m/px with origin (-10.6, -3.6), so it
# spans x in [-10.6, 10.6] and y in [-3.6, 10.1]. A goal outside those bounds is
# rejected or aborted by Nav2 -- that is exactly what the previous 97 x 73 px map
# did to every goal in the project.
MAP_BOUNDS = (-10.6, 10.6, -3.6, 10.1)
MAP_MARGIN = 0.35          # nav2_params.yaml inflation_radius


def test_every_goal_is_well_inside_the_saved_map(table):
    min_x, max_x, min_y, max_y = MAP_BOUNDS
    for key in table.keys():
        room = table.get(key)
        assert min_x + MAP_MARGIN <= room.x <= max_x - MAP_MARGIN, (
            f'{key} goal x={room.x} is outside the saved map'
        )
        assert min_y + MAP_MARGIN <= room.y <= max_y - MAP_MARGIN, (
            f'{key} goal y={room.y} is outside the saved map'
        )


def test_every_room_stays_inside_its_module(table):
    # Each module is a 5 m x 5 m room centred on these coordinates; the goal
    # poses sit slightly off-centre to clear the furniture.
    centres = {
        'command_module': (-7.0, 6.0), 'sleeping_quarters': (0.0, 6.0),
        'airlock': (7.0, 6.0), 'cargo_bay': (-7.0, 0.0),
        'medical_bay': (0.0, 0.0), 'engine_room': (7.0, 0.0),
    }
    for key, (cx, cy) in centres.items():
        room = table.get(key)
        assert abs(room.x - cx) < 2.3, f'{key} goal is outside its module in x'
        assert abs(room.y - cy) < 2.3, f'{key} goal is outside its module in y'


# ------------------------------------------------------------- name resolution
@pytest.mark.parametrize('phrase,expected', [
    ('medical_bay', 'medical_bay'),
    ('medical bay', 'medical_bay'),
    ('Medical Bay', 'medical_bay'),
    ('the medical bay', 'medical_bay'),
    ('Go to the medical bay', 'medical_bay'),
    ('medbay', 'medical_bay'),
    ('infirmary', 'medical_bay'),
    ('sick bay', 'medical_bay'),
    ('command_module', 'command_module'),
    ('command module', 'command_module'),
    ('Navigate to the command module', 'command_module'),
    ('the cockpit', 'command_module'),
    ('bridge', 'command_module'),
    ('cargo_bay', 'cargo_bay'),
    ('Go to the cargo bay', 'cargo_bay'),
    ('storage', 'cargo_bay'),
    ('the engine room', 'engine_room'),
    ('engineering', 'engine_room'),
    ('airlock', 'airlock'),
    ('air lock', 'airlock'),
    ('sleeping quarters', 'sleeping_quarters'),
    ('crew quarters', 'sleeping_quarters'),
    # Singular/short forms added with the astra_robot integration. "engine" is
    # the one the project workflow doc uses in its own example, "go fix the
    # engine", which did not resolve before.
    ('engine', 'engine_room'),
    ('go fix the engine', 'engine_room'),
    ('eva', 'airlock'),
    ('medic', 'medical_bay'),
    ('hold', 'cargo_bay'),
    ('helm', 'command_module'),
    ('bunk', 'sleeping_quarters'),
    ('thrusters', 'engine_room'),
])
def test_resolve_natural_phrases(table, phrase, expected):
    room = table.resolve(phrase)
    assert room is not None, f'{phrase!r} did not resolve to anything'
    assert room.key == expected


@pytest.mark.parametrize('typo,expected', [
    ('medcal bay', 'medical_bay'),
    ('comand module', 'command_module'),
    ('cargo bey', 'cargo_bay'),
])
def test_resolve_tolerates_small_typos(table, typo, expected):
    room = table.resolve(typo)
    assert room is not None and room.key == expected


@pytest.mark.parametrize('phrase', [
    '', '   ', None, 'the kitchen', 'observation deck', 'zzzzzz',
])
def test_unknown_destinations_return_none(table, phrase):
    # The tool turns None into "valid destinations are: ..." so the LLM can ask.
    assert table.resolve(phrase) is None


def test_normalise_strips_filler_words():
    assert normalise('Go to the Medical Bay, please') == 'medical_bay'
    assert normalise('  NAVIGATE  to   the  cargo-bay ') == 'cargo_bay'
    assert normalise('') == ''
    assert normalise(None) == ''


# --------------------------------------------------------------------- helpers
def test_nearest_room(table):
    assert table.nearest(0.1, 1.4).key == 'medical_bay'
    assert table.nearest(-6.8, 5.5).key == 'command_module'


def test_describe_lists_every_room(table):
    text = table.describe()
    for key in table.keys():
        assert key in text


def test_yaw_quaternion_round_trip():
    for yaw in (0.0, 1.5707963, -1.5707963, 3.0):
        z, w = yaw_to_quaternion(yaw)
        assert abs(quaternion_to_yaw(z, w) - yaw) < 1e-6
    # identity rotation, as used by the C++ client
    assert yaw_to_quaternion(0.0) == (0.0, 1.0)


# ----------------------------------------------------------------- bad configs
def test_missing_file_gives_a_clear_error():
    with pytest.raises(ValueError, match='not found'):
        RoomTable.from_file('/definitely/not/a/real/rooms.yaml')


def test_malformed_file_is_rejected(tmp_path):
    bad = tmp_path / 'rooms.yaml'
    bad.write_text('rooms:\n  broken_room:\n    x: 1.0\n')  # no y
    with pytest.raises(ValueError, match="missing required field 'y'"):
        RoomTable.from_file(str(bad))

    bad.write_text('destinations:\n  a: 1\n')
    with pytest.raises(ValueError, match="top-level 'rooms:'"):
        RoomTable.from_file(str(bad))

    bad.write_text('rooms:\n  a:\n    x: one\n    y: 2.0\n')
    with pytest.raises(ValueError, match='non-numeric'):
        RoomTable.from_file(str(bad))


def test_conflicting_aliases_are_rejected(tmp_path):
    bad = tmp_path / 'rooms.yaml'
    bad.write_text(
        'rooms:\n'
        '  room_a:\n    x: 0.0\n    y: 0.0\n    aliases: ["shared name"]\n'
        '  room_b:\n    x: 1.0\n    y: 1.0\n    aliases: ["shared name"]\n'
    )
    with pytest.raises(ValueError, match='claimed by both'):
        RoomTable.from_file(str(bad))
