from __future__ import annotations

from typing import Any

from ..utils import bb_get, bb_set, get_locations, make_pose_stamped


def go_to_location(node, blackboard: Any, location_name: str) -> str:
    """Navigate to a named location from blackboard['locations']."""
    locations_yaml = bb_get(blackboard, "locations", {})
    locations = get_locations(locations_yaml)

    task_params = bb_get(blackboard, "task_params", {})
    navigation_params = task_params.get("navigation", {})
    map_frame = str(navigation_params.get("map_frame", "map"))

    node.get_logger().info(f"[NAV] go_to_location: {location_name}")

    if location_name not in locations:
        node.get_logger().error(f"[NAV] location not found: {location_name}")
        node.get_logger().error(f"[NAV] available locations: {list(locations.keys())}")
        return "aborted"

    pose = make_pose_stamped(
        node=node,
        location=locations[location_name],
        frame_id=map_frame,
    )
    bb_set(blackboard, "last_nav_goal", location_name)

    ok = node.navigate_to_pose(pose)

    if ok:
        node.get_logger().info(f"[NAV] arrived or skipped: {location_name}")
        return "succeeded"

    node.get_logger().warn(f"[NAV] failed: {location_name}")
    return "retry"


def go_to_interaction_point(node, blackboard: Any) -> str:
    """Move to the point in front of the operator/user before listening."""
    task_params = bb_get(blackboard, "task_params", {})
    navigation_params = task_params.get("navigation", {})
    location_name = str(navigation_params.get("interaction_location", "interaction_point"))

    locations_yaml = bb_get(blackboard, "locations", {})
    locations = get_locations(locations_yaml)

    if location_name not in locations and "instruction_point" in locations:
        node.get_logger().warn(
            "[NAV] interaction_point not found. Falling back to instruction_point."
        )
        location_name = "instruction_point"

    return go_to_location(node, blackboard, location_name)


def go_to_destination(node, blackboard: Any) -> str:
    """Navigate to the destination extracted from LISTEN_COMMAND.

    LISTEN_COMMAND stores destination_location on the blackboard. If it is not
    available, this state can fall back to target_room + navigation.room_to_location.
    """
    task_params = bb_get(blackboard, "task_params", {})
    navigation_params = task_params.get("navigation", {})
    room_to_location = navigation_params.get("room_to_location", {})

    destination_location = bb_get(blackboard, "destination_location", "")
    target_room = bb_get(blackboard, "target_room", "")

    if not destination_location and target_room:
        destination_location = str(room_to_location.get(target_room, f"{target_room}_pose"))
        bb_set(blackboard, "destination_location", destination_location)

    if not destination_location:
        node.get_logger().error("[NAV] destination_location is empty. Did LISTEN_COMMAND parse a room?")
        return "aborted"

    node.get_logger().info(
        f"[NAV] target_room={target_room}, destination_location={destination_location}"
    )
    return go_to_location(node, blackboard, destination_location)

def return_to_interaction_point(node, blackboard: Any) -> str:
    """Return to the interaction point after successfully grasping an object."""
    node.get_logger().info("[NAV] return to interaction point for handover")
    return go_to_interaction_point(node, blackboard)


def go_to_door_exit(node, blackboard: Any) -> str:
    """Move to the configured waiting point in front of the exit door."""
    task_params = bb_get(blackboard, "task_params", {})
    exit_params = task_params.get("exit_arena", {})
    location_name = str(exit_params.get("exit_location", "door_exit"))

    node.get_logger().info(f"[NAV] move to exit-door point: {location_name}")
    return go_to_location(node, blackboard, location_name)

def go_to_door_enter(node, blackboard: Any) -> str:
    """Move through the opened door to the configured final waypoint."""
    task_params = bb_get(blackboard, "task_params", {})
    exit_params = task_params.get("exit_arena", {})
    location_name = str(exit_params.get("door_enter_location", "door_enter"))

    node.get_logger().info(f"[NAV] move through opened door to: {location_name}")
    return go_to_location(node, blackboard, location_name)

