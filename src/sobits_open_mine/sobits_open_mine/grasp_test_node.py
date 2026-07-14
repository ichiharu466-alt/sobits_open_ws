from __future__ import annotations

from typing import Any

import rclpy
from yasmin import Blackboard

from .bring_me_node import BringMeNode
from .task_actions.grasp import grasp_object
from .utils import bb_set, load_yaml


class GraspTestNode(BringMeNode):
    """Manual grasp test node using hsrb_library.

    Start hsrb_library first:
      ros2 launch hsrb_library library_server.launch.py

    Then run e.g.:
      ros2 run sobits_open_mine grasp_test_node --ros-args \
        -p target_x:=0.70 -p target_y:=0.0 -p target_z:=0.55
    """

    def __init__(self):
        super().__init__(enable_nav=False)
        self.declare_parameter("target_object", "test_object")
        self.declare_parameter("target_x", 0.70)
        self.declare_parameter("target_y", 0.00)
        self.declare_parameter("target_z", 0.55)
        self.declare_parameter("task_params_file", "task_params.yaml")

        self.target_object = str(self.get_parameter("target_object").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.target_z = float(self.get_parameter("target_z").value)
        self.task_params_file = str(self.get_parameter("task_params_file").value)

        self.get_logger().info("GraspTestNode initialized for hsrb_library grasp.")
        self.get_logger().info(
            f"manual target_pose_base: x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}"
        )

    def run_test(self) -> str:
        blackboard = Blackboard()
        task_params = load_yaml(self.task_params_file, package_name="sobits_open_mine", subdir="config")
        blackboard["task_params"] = task_params
        bb_set(blackboard, "target_object", self.target_object)
        bb_set(
            blackboard,
            "target_pose_base",
            {
                "frame_id": "base_footprint",
                "x": self.target_x,
                "y": self.target_y,
                "z": self.target_z,
            },
        )
        bb_set(
            blackboard,
            "target_pose_camera",
            {
                "frame_id": "manual_test",
                "x": self.target_x,
                "y": self.target_y,
                "z": self.target_z,
            },
        )
        result = grasp_object(self, blackboard)
        self.get_logger().info(f"grasp_object result: {result}")
        return result


def main(args: Any = None):
    rclpy.init(args=args)
    node = GraspTestNode()
    try:
        node.run_test()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
