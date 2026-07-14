from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..utils import normalize_text


@dataclass
class ParsedCommand:
    raw_text: str
    target_object: str = ""
    target_room: str = ""
    destination_location: str = ""
    yolo_target_labels: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.target_object and self.target_room and self.destination_location)


def _find_alias(text_norm: str, aliases: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    """Return the first canonical name whose words appear in the command."""
    for canonical, spec in aliases.items():
        words = spec.get("words", []) if isinstance(spec, dict) else []
        candidates = [canonical, *words]
        for word in candidates:
            word_norm = normalize_text(str(word))
            if word_norm and word_norm in text_norm:
                return canonical, spec if isinstance(spec, dict) else {}

    return "", {}


def parse_command(command_text: str, task_params: dict[str, Any]) -> ParsedCommand:
    """Parse a simple Bring-Me command into object and room.

    This intentionally stays lightweight and rule-based so it is predictable on
    the robot. Configure aliases in config/task_params.yaml.
    """
    command = ParsedCommand(raw_text=command_text)
    text_norm = normalize_text(command_text)

    command_params = task_params.get("command", {})
    room_aliases = command_params.get("rooms", {})
    object_aliases = command_params.get("objects", {})

    room_name, room_spec = _find_alias(text_norm, room_aliases)
    object_name, object_spec = _find_alias(text_norm, object_aliases)

    command.target_room = room_name
    command.target_object = object_name

    if room_name:
        command.destination_location = str(room_spec.get("location", f"{room_name}_pose"))

    if object_name:
        yolo_labels = object_spec.get("yolo_labels", [])
        if isinstance(yolo_labels, list):
            command.yolo_target_labels = [str(label) for label in yolo_labels if str(label)]
        else:
            command.yolo_target_labels = [str(yolo_labels)]

        if object_name not in command.yolo_target_labels:
            command.yolo_target_labels.insert(0, object_name)

    return command
