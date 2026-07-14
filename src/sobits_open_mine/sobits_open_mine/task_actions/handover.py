from __future__ import annotations

from typing import Any
import time

from ..utils import bb_get, bb_set
from .grasp import _move_joint, _open_gripper
from .perception import turn_to_human


def _handover_cfg(blackboard: Any) -> dict[str, Any]:
    return bb_get(blackboard, "task_params", {}).get("handover", {})


def _grasp_cfg(blackboard: Any) -> dict[str, Any]:
    return bb_get(blackboard, "task_params", {}).get("grasp", {})


def _move_to_handover_pose(node, blackboard: Any) -> bool:
    cfg = _handover_cfg(blackboard)

    joints = cfg.get(
        "handover_joint_names",
        [
            "arm_lift_joint",
            "arm_flex_joint",
            "arm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
        ],
    )
    values = cfg.get(
        "handover_joint_rad",
        [0.25, -0.35, 0.0, 0.0, 0.0],
    )

    if not isinstance(joints, list) or not isinstance(values, list) or len(joints) != len(values):
        node.get_logger().error("[DELIVER] invalid handover_joint_names / handover_joint_rad")
        return False

    node.get_logger().info("[DELIVER] move arm to handover pose")
    return _move_joint(
        node,
        [str(j) for j in joints],
        [float(v) for v in values],
        duration_sec=float(cfg.get("handover_motion_duration_sec", 3.0)),
    )


def _return_after_handover(node, blackboard: Any) -> None:
    cfg = _handover_cfg(blackboard)
    joints = cfg.get(
        "return_joint_names",
        [
            "arm_flex_joint",
            "arm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
            "arm_lift_joint",
        ],
    )
    values = cfg.get(
        "return_joint_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0],
    )

    if not isinstance(joints, list) or not isinstance(values, list) or len(joints) != len(values):
        node.get_logger().warn("[DELIVER] invalid return pose config. Skip return motion.")
        return

    node.get_logger().info("[DELIVER] return arm after handover")
    _move_joint(
        node,
        [str(j) for j in joints],
        [float(v) for v in values],
        duration_sec=float(cfg.get("return_motion_duration_sec", 3.0)),
    )


def deliver_object(node, blackboard: Any) -> str:
    """Turn to a human, ask them to receive the object, count down, then release.

    Flow:
      1. Detect/center the human using the HUMAN_DETECTION behavior.
      2. Move arm to a handover pose.
      3. Speak 「受け取ってください」.
      4. Count down 3, 2, 1.
      5. Open gripper to release.
      6. Return arm to a safe pose.
    """
    target_object = bb_get(blackboard, "target_object", "object")
    cfg = _handover_cfg(blackboard)

    node.get_logger().info("========== DELIVER_OBJECT ==========")
    node.get_logger().info(f"[DELIVER] target_object={target_object}")

    if bool(cfg.get("turn_to_human_before_handover", True)):
        result = turn_to_human(node, blackboard)
        if result != "succeeded":
            node.get_logger().warn(f"[DELIVER] turn_to_human result={result}")
            if bool(cfg.get("require_human_detection", False)):
                return result

    if bool(cfg.get("use_handover_pose", True)):
        if not _move_to_handover_pose(node, blackboard):
            return "retry"

    phrase = str(cfg.get("request_phrase", "受け取ってください"))
    node.say(phrase)

    countdown_start = int(cfg.get("countdown_start", 3))
    countdown_interval = float(cfg.get("countdown_interval_sec", 1.0))
    for n in range(countdown_start, 0, -1):
        node.say(str(n))
        time.sleep(max(0.0, countdown_interval))

    release_phrase = str(cfg.get("release_phrase", "離します"))
    if release_phrase:
        node.say(release_phrase)

    grasp_cfg = _grasp_cfg(blackboard)
    if not _open_gripper(node, grasp_cfg):
        node.get_logger().warn("[DELIVER] failed to publish open gripper command")
        return "retry"

    bb_set(blackboard, "object_delivered", True)

    if bool(cfg.get("return_arm_after_handover", True)):
        _return_after_handover(node, blackboard)

    node.get_logger().info("[DELIVER] succeeded")
    return "succeeded"
