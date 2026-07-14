from __future__ import annotations

from yasmin import StateMachine

from .states import (
    AbortState,
    DeliverObjectState,
    DoorOpenState,
    EstimateObjectPoseState,
    ExitArenaState,
    ExitDoorOpenState,
    GoToDoorEnterState,
    GoToDestinationState,
    GoToDoorExitState,
    GoToInteractionPointState,
    GraspObjectState,
    HumanDetectionState,
    ListenCommandState,
    ObjectDetectionState,
    ReturnToInteractionPointState,
    SkipState,
    StartState,
)


def maybe_skip(node, state_name: str, state, skip_states: set[str]):
    if state_name in skip_states:
        return SkipState(node, state_name)
    return state


def create_state_machine(
    node,
    skip_states: set[str] | None = None,
) -> StateMachine:
    if skip_states is None:
        skip_states = set()

    sm = StateMachine(outcomes=["succeeded", "aborted"])

    sm.add_state("START", StartState(node), transitions={"succeeded": "DOOR_OPEN"})

    sm.add_state(
        "DOOR_OPEN",
        maybe_skip(node, "DOOR_OPEN", DoorOpenState(node), skip_states),
        transitions={
            "succeeded": "GO_TO_INTERACTION_POINT",
            "retry": "DOOR_OPEN",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "GO_TO_INTERACTION_POINT",
        maybe_skip(node, "GO_TO_INTERACTION_POINT", GoToInteractionPointState(node), skip_states),
        transitions={
            "succeeded": "HUMAN_DETECTION",
            "retry": "GO_TO_INTERACTION_POINT",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "HUMAN_DETECTION",
        maybe_skip(node, "HUMAN_DETECTION", HumanDetectionState(node), skip_states),
        transitions={
            "succeeded": "LISTEN_COMMAND",
            "retry": "HUMAN_DETECTION",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "LISTEN_COMMAND",
        maybe_skip(node, "LISTEN_COMMAND", ListenCommandState(node), skip_states),
        transitions={
            "succeeded": "GO_TO_DESTINATION",
            "retry": "LISTEN_COMMAND",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "GO_TO_DESTINATION",
        maybe_skip(node, "GO_TO_DESTINATION", GoToDestinationState(node), skip_states),
        transitions={
            "succeeded": "OBJECT_DETECTION",
            "retry": "GO_TO_DESTINATION",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "OBJECT_DETECTION",
        maybe_skip(node, "OBJECT_DETECTION", ObjectDetectionState(node), skip_states),
        transitions={
            "succeeded": "ESTIMATE_OBJECT_POSE",
            "retry": "OBJECT_DETECTION",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "ESTIMATE_OBJECT_POSE",
        maybe_skip(node, "ESTIMATE_OBJECT_POSE", EstimateObjectPoseState(node), skip_states),
        transitions={
            "succeeded": "GRASP_OBJECT",
            "retry": "ESTIMATE_OBJECT_POSE",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "GRASP_OBJECT",
        maybe_skip(node, "GRASP_OBJECT", GraspObjectState(node), skip_states),
        transitions={
            "succeeded": "RETURN_TO_INTERACTION_POINT",
            "retry": "GRASP_OBJECT",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "RETURN_TO_INTERACTION_POINT",
        maybe_skip(
            node,
            "RETURN_TO_INTERACTION_POINT",
            ReturnToInteractionPointState(node),
            skip_states,
        ),
        transitions={
            "succeeded": "DELIVER_OBJECT",
            "retry": "RETURN_TO_INTERACTION_POINT",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "DELIVER_OBJECT",
        maybe_skip(node, "DELIVER_OBJECT", DeliverObjectState(node), skip_states),
        transitions={
            "succeeded": "GO_TO_DOOR_EXIT",
            "retry": "DELIVER_OBJECT",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "GO_TO_DOOR_EXIT",
        maybe_skip(node, "GO_TO_DOOR_EXIT", GoToDoorExitState(node), skip_states),
        transitions={
            "succeeded": "EXIT_DOOR_OPEN",
            "retry": "GO_TO_DOOR_EXIT",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "EXIT_DOOR_OPEN",
        maybe_skip(node, "EXIT_DOOR_OPEN", ExitDoorOpenState(node), skip_states),
        transitions={
            "succeeded": "GO_TO_DOOR_ENTER",
            "retry": "EXIT_DOOR_OPEN",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "GO_TO_DOOR_ENTER",
        maybe_skip(node, "GO_TO_DOOR_ENTER", GoToDoorEnterState(node), skip_states),
        transitions={
            "succeeded": "succeeded",
            "retry": "GO_TO_DOOR_ENTER",
            "aborted": "ABORT",
        },
    )

    sm.add_state(
        "EXIT_ARENA",
        maybe_skip(node, "EXIT_ARENA", ExitArenaState(node), skip_states),
        transitions={
            "succeeded": "succeeded",
            "retry": "EXIT_ARENA",
            "aborted": "ABORT",
        },
    )

    sm.add_state("ABORT", AbortState(node), transitions={"aborted": "aborted"})

    return sm
