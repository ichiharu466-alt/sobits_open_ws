from __future__ import annotations

import argparse

import rclpy
from rclpy.utilities import remove_ros_args
from yasmin import Blackboard

from .bring_me_node import BringMeNode
from .state_machine import create_state_machine
from .utils import bb_get, load_yaml


PACKAGE_NAME = "sobits_open_mine"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the Bring Me state machine."
    )

    parser.add_argument(
        "--skip-states",
        nargs="*",
        default=[],
        help="State names to skip. Example: --skip-states DOOR_OPEN HUMAN_DETECTION",
    )
    parser.add_argument(
        "--enable-nav",
        action="store_true",
        help="Enable real navigation action calls. Without this, navigation is log-only unless launch parameter enables it.",
    )
    parser.add_argument(
        "--target-object",
        default="sponge",
        help="Target object name stored in the blackboard for later task_actions use.",
    )
    parser.add_argument(
        "--location-file",
        default="locations.yaml",
        help="Location YAML file name in config/ or an absolute path.",
    )
    parser.add_argument(
        "--task-params-file",
        default="task_params.yaml",
        help="Task parameter YAML file name in config/ or an absolute path.",
    )

    return parser.parse_args(remove_ros_args()[1:])


def main() -> None:
    args = parse_args()

    rclpy.init()

    # If --enable-nav is not supplied, leave enable_nav=None so ROS parameters from launch can still work.
    node = BringMeNode(enable_nav=True if args.enable_nav else None)

    try:
        locations = load_yaml(
            args.location_file,
            package_name=PACKAGE_NAME,
            subdir="config",
        )
        task_params = load_yaml(
            args.task_params_file,
            package_name=PACKAGE_NAME,
            subdir="config",
        )

        blackboard = Blackboard()
        blackboard["locations"] = locations
        blackboard["task_params"] = task_params
        blackboard["skip_states"] = set(args.skip_states)
        blackboard["target_object"] = args.target_object

        node.get_logger().info("========================================")
        node.get_logger().info("Bring Me StateMachine")
        node.get_logger().info(f"skip_states: {sorted(args.skip_states)}")
        node.get_logger().info(f"target_object: {args.target_object}")
        node.get_logger().info(f"location_file: {args.location_file}")
        node.get_logger().info(f"task_params_file: {args.task_params_file}")
        node.get_logger().info("========================================")

        sm = create_state_machine(
            node=node,
            skip_states=set(args.skip_states),
        )

        outcome = sm(blackboard)
        node.get_logger().info(f"StateMachine finished with outcome: {outcome}")

        summary_keys = [
            "command_text",
            "target_room",
            "target_object",
            "destination_location",
            "yolo_target_labels",
            "human_detected",
            "last_nav_goal",
            "target_detection",
            "target_pose_camera",
            "grasp_plan",
            "object_grasped",
        ]
        node.get_logger().info("========== Blackboard summary ==========")
        for key in summary_keys:
            value = bb_get(blackboard, key, None)
            if value is not None:
                node.get_logger().info(f"{key}: {value}")
        node.get_logger().info("========================================")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
