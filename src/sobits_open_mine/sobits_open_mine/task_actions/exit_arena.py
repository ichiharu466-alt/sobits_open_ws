from __future__ import annotations

from typing import Any

from ..utils import bb_get
from .door import wait_for_door_open
from .navigate import go_to_location


def exit_arena(node, blackboard: Any) -> str:
    """Ask/wait for the exit door to open, then navigate out of the arena."""
    cfg = bb_get(blackboard, "task_params", {}).get("exit_arena", {})
    exit_location = str(cfg.get("exit_location", "door_exit"))

    node.get_logger().info("========== EXIT_ARENA ==========")

    if bool(cfg.get("wait_for_exit_door_open", True)):
        node.say(str(cfg.get("door_request_phrase", "最後にドアを開けてください")))
        door_result = wait_for_door_open(node, blackboard)
        if door_result != "succeeded":
            node.get_logger().warn(f"[EXIT] door open result={door_result}")
            return door_result

    node.say(str(cfg.get("exit_phrase", "アリーナから出ます")))
    return go_to_location(node, blackboard, exit_location)
