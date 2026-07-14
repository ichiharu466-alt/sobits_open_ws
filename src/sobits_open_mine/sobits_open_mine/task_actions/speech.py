from __future__ import annotations

from typing import Any

from ..utils import bb_get, bb_set
from .command_parser import parse_command


def listen_command(node, blackboard: Any) -> str:
    params = bb_get(blackboard, "task_params", {}).get("speech", {})
    task_params = bb_get(blackboard, "task_params", {})

    listen_timeout_sec = int(params.get("listen_timeout_sec", 7))
    silent_mode = bool(params.get("silent_mode", False))
    feedback_rate = float(params.get("feedback_rate", 0.5))
    retry_when_empty = bool(params.get("retry_when_empty", True))
    retry_when_parse_failed = bool(params.get("retry_when_parse_failed", True))

    node.get_logger().info("[SPEECH] start listening command")
    node.say("指示をお願いします")

    text = node.listen(
        timeout_sec=listen_timeout_sec,
        silent_mode=silent_mode,
        feedback_rate=feedback_rate,
    )

    if text is None:
        node.get_logger().warn("[SPEECH] speech recognition failed")
        return "retry" if retry_when_empty else "aborted"

    text = text.strip()
    if not text:
        node.get_logger().warn("[SPEECH] empty command")
        return "retry" if retry_when_empty else "aborted"

    parsed = parse_command(text, task_params)

    bb_set(blackboard, "command_text", text)
    bb_set(blackboard, "target_object", parsed.target_object)
    bb_set(blackboard, "target_room", parsed.target_room)
    bb_set(blackboard, "destination_location", parsed.destination_location)
    bb_set(blackboard, "yolo_target_labels", parsed.yolo_target_labels)

    node.get_logger().info(f"[SPEECH] command_text: {text}")
    node.get_logger().info(
        "[SPEECH] parsed: "
        f"object={parsed.target_object}, room={parsed.target_room}, "
        f"destination={parsed.destination_location}, yolo_labels={parsed.yolo_target_labels}"
    )

    if not parsed.ok:
        node.get_logger().warn("[SPEECH] command parse failed")
        node.say("すみません。もう一度お願いします")
        return "retry" if retry_when_parse_failed else "aborted"

    node.say(f"{parsed.target_room}の{parsed.target_object}ですね")
    return "succeeded"
