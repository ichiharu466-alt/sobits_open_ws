from __future__ import annotations

from typing import Any

from yasmin import State

from .task_actions.door import wait_for_door_open
from .task_actions.grasp import grasp_object
from .task_actions.handover import deliver_object
from .task_actions.exit_arena import exit_arena
from .task_actions.navigate import (
    go_to_destination,
    go_to_door_exit,
    go_to_door_enter,
    go_to_interaction_point,
    return_to_interaction_point,
)
from .task_actions.perception import (
    detect_target_object,
    estimate_object_pose_from_depth,
    turn_to_human,
)
from .task_actions.speech import listen_command


class LogState(State):
    """Log-only state for steps that are not implemented yet."""

    def __init__(self, node, state_name: str):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node
        self.state_name = state_name

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info(f"[STATE] {self.state_name}: start")
        self.node.get_logger().info(f"[STATE] {self.state_name}: succeeded")
        return "succeeded"


class StartState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] START: start")
        self.node.get_logger().info("[STATE] START: succeeded")
        return "succeeded"


class SkipState(State):
    def __init__(self, node, state_name: str):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node
        self.state_name = state_name

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().warn(f"[STATE] {self.state_name}: skipped")
        return "succeeded"


class DoorOpenState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] DOOR_OPEN: start")
        return wait_for_door_open(self.node, blackboard)


class GoToInteractionPointState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] GO_TO_INTERACTION_POINT: start")
        return go_to_interaction_point(self.node, blackboard)


class ReturnToInteractionPointState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info(
            "[STATE] RETURN_TO_INTERACTION_POINT: start"
        )
        return return_to_interaction_point(self.node, blackboard)


class GoToDoorExitState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] GO_TO_DOOR_EXIT: start")
        return go_to_door_exit(self.node, blackboard)


class ExitDoorOpenState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] EXIT_DOOR_OPEN: start")
        return wait_for_door_open(self.node, blackboard)


class GoToDoorEnterState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] GO_TO_DOOR_ENTER: start")
        return go_to_door_enter(self.node, blackboard)


class HumanDetectionState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] HUMAN_DETECTION: start")
        return turn_to_human(self.node, blackboard)


class ListenCommandState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] LISTEN_COMMAND: start")
        return listen_command(self.node, blackboard)


class GoToDestinationState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] GO_TO_DESTINATION: start")
        return go_to_destination(self.node, blackboard)


class ObjectDetectionState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] OBJECT_DETECTION: start")
        return detect_target_object(self.node, blackboard)


class EstimateObjectPoseState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] ESTIMATE_OBJECT_POSE: start")
        return estimate_object_pose_from_depth(self.node, blackboard)


class GraspObjectState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] GRASP_OBJECT: start")
        return grasp_object(self.node, blackboard)


class DeliverObjectState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] DELIVER_OBJECT: start")
        return deliver_object(self.node, blackboard)


class ExitArenaState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "retry", "aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.get_logger().info("[STATE] EXIT_ARENA: start")
        return exit_arena(self.node, blackboard)


class AbortState(State):
    def __init__(self, node):
        super().__init__(outcomes=["aborted"])
        self.node = node

    def execute(self, blackboard: Any) -> str:
        self.node.stop_robot()
        self.node.get_logger().error("[STATE] ABORT: aborted")
        return "aborted"
