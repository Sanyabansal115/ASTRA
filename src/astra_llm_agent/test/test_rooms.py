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
def test_all_six_rooms_from_the_cpp_client_are_present(table):
    expected = {
        'command_module', 'sleeping_quarters', 'airlock',
        'cargo_bay', 'medical_bay', 'engine_room',
    }
    assert set(table.keys()) == expected


def test_coordinates_match_the_cpp_client_where_they_are_obstacle_free(table):
    # navigation_client.cpp lines 14-21; the three bottom-row goals are
    # deliberately offset in y (documented in rooms.yaml).
    assert (table.get('command_module').x, table.get('command_module').y) == (-7.0, 6.0)
    assert (table.get('sleeping_quarters').x, table.get('sleeping_quarters').y) == (0.0, 6.0)
    assert (table.get('airlock').x, table.get('airlock').y) == (7.0, 6.0)
    assert table.get('cargo_bay').x == -7.0
    assert table.get('medical_bay').x == 0.0
    assert table.get('engine_room').x == 7.0


def test_offset_goals_clear_the_static_obstacles(table):
    # crate3 (0.6 m box) at (-7, 0), engine_machine (0.6 m radius) at (7, 0),
    # hospital beds either side of (0, 0). Nav2 uses robot_radius 0.15 and
    # inflation_radius 0.5, so keep at least ~0.9 m from those centres.
    for key, obstacle in (
        ('cargo_bay', (-7.0, 0.0)),
        ('engine_room', (7.0, 0.0)),
        ('medical_bay', (0.0, 0.0)),
    ):
        room = table.get(key)
        distance = ((room.x - obstacle[0]) ** 2 + (room.y - obstacle[1]) ** 2) ** 0.5
        assert distance >= 0.9, f'{key} goal is too close to the obstacle at {obstacle}'


def test_every_room_stays_inside_its_5x5_module(table):
    # Each module is a 5 m x 5 m room centred on the C++ coordinate.
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
