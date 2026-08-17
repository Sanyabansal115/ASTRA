"""Semantic location table for the ASTRA LLM navigation agent.

This module deliberately contains **no ROS imports** so that the room lookup
logic can be unit-tested with plain ``pytest`` (see ``test/test_rooms.py``)
without a running ROS 2 installation.

It provides the "semantic-to-coordinate mapping" layer described in the
project brief: room names spoken by the user (``"the medical bay"``,
``"medbay"``, ``"infirmary"``) are resolved to a goal pose in the ``map``
frame that is handed to Nav2.
"""

from __future__ import annotations

import difflib
import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml


# Words that carry no meaning when naming a destination. Stripping them lets
# "go to the medical bay please" resolve to the key "medical_bay" even if the
# LLM passes the whole phrase through instead of just the room name.
_FILLER_WORDS = {
    "go", "goto", "navigate", "move", "drive", "head", "travel", "please",
    "the", "a", "an", "to", "towards", "toward", "into", "in", "at", "robot",
    "over", "now",
}


@dataclass(frozen=True)
class Room:
    """One semantic destination and its goal pose in the ``map`` frame."""

    key: str
    x: float
    y: float
    yaw: float = 0.0
    description: str = ""
    aliases: List[str] = field(default_factory=list)

    @property
    def label(self) -> str:
        """Human-readable name, e.g. ``medical_bay`` -> ``Medical Bay``."""
        return self.key.replace("_", " ").title()


class RoomTable:
    """Loaded ``rooms.yaml`` plus the fuzzy name matching used by the tools."""

    def __init__(self, rooms: Dict[str, Room]):
        if not rooms:
            raise ValueError("rooms.yaml contains no rooms")
        self._rooms = rooms
        self._alias_index = self._build_alias_index(rooms)

    # ------------------------------------------------------------------ load
    @classmethod
    def from_file(cls, path: str) -> "RoomTable":
        """Load and validate a rooms.yaml file. Raises ValueError if unusable."""
        try:
            with open(path, "r", encoding="utf-8") as handle:
                raw = yaml.safe_load(handle)
        except FileNotFoundError as exc:
            raise ValueError(f"Room file not found: {path}") from exc
        except yaml.YAMLError as exc:
            raise ValueError(f"Room file {path} is not valid YAML: {exc}") from exc

        if not isinstance(raw, dict) or "rooms" not in raw:
            raise ValueError(f"Room file {path} must contain a top-level 'rooms:' mapping")

        entries = raw["rooms"]
        if not isinstance(entries, dict) or not entries:
            raise ValueError(f"Room file {path} has an empty or malformed 'rooms:' mapping")

        rooms: Dict[str, Room] = {}
        for key, value in entries.items():
            if not isinstance(value, dict):
                raise ValueError(f"Room '{key}' must be a mapping with at least x and y")
            for coord in ("x", "y"):
                if coord not in value:
                    raise ValueError(f"Room '{key}' is missing required field '{coord}'")
            try:
                x = float(value["x"])
                y = float(value["y"])
                yaw = float(value.get("yaw", 0.0))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Room '{key}' has a non-numeric x/y/yaw value") from exc
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError(f"Room '{key}' has a non-finite coordinate")

            aliases = value.get("aliases") or []
            if isinstance(aliases, str):
                aliases = [aliases]
            if not isinstance(aliases, list):
                raise ValueError(f"Room '{key}' has an 'aliases' field that is not a list")

            rooms[str(key)] = Room(
                key=str(key),
                x=x,
                y=y,
                yaw=yaw,
                description=str(value.get("description", "")),
                aliases=[str(a) for a in aliases],
            )

        return cls(rooms)

    @staticmethod
    def _build_alias_index(rooms: Dict[str, Room]) -> Dict[str, str]:
        """Map every normalised alias to its canonical room key.

        Conflicting aliases are a configuration error we want to hear about at
        start-up rather than half-way through a demo, so this raises.
        """
        index: Dict[str, str] = {}
        for room in rooms.values():
            for name in [room.key, *room.aliases]:
                norm = normalise(name)
                if not norm:
                    continue
                owner = index.get(norm)
                if owner is not None and owner != room.key:
                    raise ValueError(
                        f"Alias '{name}' is claimed by both '{owner}' and '{room.key}'"
                    )
                index[norm] = room.key
        return index

    # ----------------------------------------------------------------- query
    @property
    def rooms(self) -> Dict[str, Room]:
        return dict(self._rooms)

    def keys(self) -> List[str]:
        return list(self._rooms)

    def get(self, key: str) -> Optional[Room]:
        return self._rooms.get(key)

    def resolve(self, query: str) -> Optional[Room]:
        """Best-effort match of free text to a room. None if nothing matches.

        Tried in order: exact key, known alias, alias contained in the phrase,
        then a fuzzy match. Returning None (rather than guessing wildly) lets
        the tool hand the valid list back to the LLM so it can ask the user.
        """
        if not query or not str(query).strip():
            return None

        norm = normalise(query)
        if not norm:
            return None

        # 1. exact canonical key or known alias
        key = self._alias_index.get(norm)
        if key is not None:
            return self._rooms[key]

        # 2. an alias appearing inside a longer phrase ("take me to the med bay")
        #    longest alias first so "medical_bay" beats a shorter partial match
        for alias in sorted(self._alias_index, key=len, reverse=True):
            if f"_{alias}_" in f"_{norm}_":
                return self._rooms[self._alias_index[alias]]

        # 3. fuzzy match for typos ("medcal bay")
        close = difflib.get_close_matches(norm, list(self._alias_index), n=1, cutoff=0.75)
        if close:
            return self._rooms[self._alias_index[close[0]]]

        return None

    def nearest(self, x: float, y: float) -> Optional[Room]:
        """Room whose goal pose is closest to (x, y) - used by get_robot_pose."""
        if not self._rooms:
            return None
        return min(self._rooms.values(), key=lambda r: math.hypot(r.x - x, r.y - y))

    def describe(self) -> str:
        """Multi-line listing used both by list_rooms() and the system prompt."""
        lines = []
        for room in self._rooms.values():
            text = f"- {room.key} ({room.label}) at x={room.x:.2f}, y={room.y:.2f}"
            if room.description:
                text += f" - {room.description}"
            lines.append(text)
        return "\n".join(lines)


def normalise(text: str) -> str:
    """Lower-case, drop punctuation and filler words, join with underscores.

    ``"Go to the Medical Bay, please"`` -> ``"medical_bay"``
    """
    if text is None:
        return ""
    lowered = re.sub(r"[^a-z0-9]+", " ", str(text).lower())
    words = [w for w in lowered.split() if w and w not in _FILLER_WORDS]
    return "_".join(words)


def yaw_to_quaternion(yaw: float):
    """Yaw (radians) -> (z, w) of a quaternion with zero roll and pitch.

    Done by hand so the package does not need transforms3d just for this.
    """
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(z: float, w: float, x: float = 0.0, y: float = 0.0) -> float:
    """Extract the yaw angle (radians) from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
