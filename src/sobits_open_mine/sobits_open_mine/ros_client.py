from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from sobits_interfaces.action import SpeechRecognition, TextToSpeech

from geometry_msgs.msg import Point, TransformStamped
from sobits_interfaces.action import MoveJoint, MoveToPose, MoveWheelLinear, MoveWheelRotate
from sobits_interfaces.srv import GetHandToTargetCoord, GetHandToTargetTF


class SpeechClient:
    def __init__(self, node: Node, action_name: str, wait_timeout_sec: float = 5.0):
        self.node = node
        self.client = ActionClient(node, SpeechRecognition, action_name)
        self.wait_timeout_sec = wait_timeout_sec

    def listen(
        self,
        timeout_sec: int | float = 7,
        silent_mode: bool = False,
        feedback_rate: float = 0.5,
    ) -> Optional[str]:
        if not self.client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().error("STT action server is not available.")
            return None

        goal = SpeechRecognition.Goal()
        goal.timeout_sec = int(timeout_sec)
        goal.silent_mode = bool(silent_mode)
        goal.feedback_rate = float(feedback_rate)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("STT goal rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        return getattr(result, "result_text", "")


class TTSClient:
    def __init__(self, node: Node, action_name: str, wait_timeout_sec: float = 5.0):
        self.node = node
        self.client = ActionClient(node, TextToSpeech, action_name)
        self.wait_timeout_sec = wait_timeout_sec

    def say(self, text: str) -> bool:
        if not text:
            return True

        self.node.get_logger().info(f"TTS: {text}")
        if not self.client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().warn("TTS action server is not available. Logging only.")
            return False

        goal = TextToSpeech.Goal()
        goal.text = text

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("TTS goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        return bool(getattr(result, "success", True))


class NavClient:
    def __init__(self, node: Node, action_name: str, wait_timeout_sec: float = 5.0):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.wait_timeout_sec = wait_timeout_sec

    def go_to(self, pose, timeout_sec: float = 90.0) -> bool:
        if not self.client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().error("Nav2 NavigateToPose action server is not available.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Navigation goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.node.get_logger().error("Navigation timed out; canceling goal.")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel_future)
                return False

        status = result_future.result().status
        self.node.get_logger().info(f"Navigation finished with status={status}")
        # action_msgs/GoalStatus STATUS_SUCCEEDED = 4
        return status == 4


class HsrbLibraryClient:
    """Small wrapper around hsrb_library actions/services.

    hsrb_library must be running separately:
      ros2 launch hsrb_library library_server.launch.py

    This wrapper intentionally avoids MoveIt.  It uses the SOBITS/HSRB action
    servers that convert named poses, joint goals, and target coordinates into
    controller commands.
    """

    def __init__(self, node: Node, wait_timeout_sec: float = 3.0):
        self.node = node
        self.wait_timeout_sec = float(wait_timeout_sec)
        self.move_pose_client = ActionClient(node, MoveToPose, "/hsrb/move_to_pose")
        self.move_joint_client = ActionClient(node, MoveJoint, "/hsrb/move_joint")
        self.move_wheel_linear_client = ActionClient(node, MoveWheelLinear, "/hsrb/move_wheel_linear")
        self.move_wheel_rotate_client = ActionClient(node, MoveWheelRotate, "/hsrb/move_wheel_rotate")
        self.hand_coord_client = node.create_client(GetHandToTargetCoord, "/hsrb/get_hand_to_coord")
        self.hand_tf_client = node.create_client(GetHandToTargetTF, "/hsrb/get_hand_to_tf")

    def move_to_pose(self, pose_name: str, duration_sec: float = 3.0) -> bool:
        if not self.move_pose_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().error("[HSRB] /hsrb/move_to_pose is not available")
            return False
        goal = MoveToPose.Goal()
        goal.pose_name = str(pose_name)
        goal.time_allowance.sec = int(duration_sec)
        goal.time_allowance.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        return self._send_action_goal(self.move_pose_client, goal, timeout_sec=duration_sec + 5.0, label=f"pose:{pose_name}")

    def move_joint(self, joint_names: list[str], joint_rads: list[float], duration_sec: float = 3.0) -> bool:
        if not self.move_joint_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().error("[HSRB] /hsrb/move_joint is not available")
            return False
        if len(joint_names) != len(joint_rads):
            self.node.get_logger().error(f"[HSRB] joint length mismatch: {joint_names} vs {joint_rads}")
            return False
        goal = MoveJoint.Goal()
        goal.target_joint_names = [str(v) for v in joint_names]
        goal.target_joint_rad = [float(v) for v in joint_rads]
        goal.time_allowance.sec = int(duration_sec)
        goal.time_allowance.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        return self._send_action_goal(self.move_joint_client, goal, timeout_sec=duration_sec + 5.0, label="move_joint")

    def move_wheel_linear(self, distance_x: float, timeout_sec: float = 8.0) -> bool:
        if not self.move_wheel_linear_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().warn("[HSRB] /hsrb/move_wheel_linear is not available")
            return False
        goal = MoveWheelLinear.Goal()
        goal.target_point = Point(x=float(distance_x), y=0.0, z=0.0)
        return self._send_action_goal(self.move_wheel_linear_client, goal, timeout_sec=timeout_sec, label=f"wheel_linear:{distance_x:.3f}")

    def move_wheel_rotate(self, yaw_rad: float, timeout_sec: float = 8.0) -> bool:
        if not self.move_wheel_rotate_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().warn("[HSRB] /hsrb/move_wheel_rotate is not available")
            return False
        goal = MoveWheelRotate.Goal()
        goal.target_yaw = float(yaw_rad)
        return self._send_action_goal(self.move_wheel_rotate_client, goal, timeout_sec=timeout_sec, label=f"wheel_rotate:{yaw_rad:.3f}")

    def get_hand_to_coord(self, x: float, y: float, z: float, frame_id: str = "base_footprint"):
        if not self.hand_coord_client.wait_for_service(timeout_sec=self.wait_timeout_sec):
            self.node.get_logger().error("[HSRB] /hsrb/get_hand_to_coord is not available")
            return None
        req = GetHandToTargetCoord.Request()
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = str(frame_id)
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = float(z)
        tf.transform.rotation.w = 1.0
        req.target_coord = tf
        future = self.hand_coord_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        res = future.result()
        if res is None:
            self.node.get_logger().error("[HSRB] get_hand_to_coord returned None")
            return None
        if not bool(res.success):
            self.node.get_logger().error(f"[HSRB] get_hand_to_coord failed: {res.message}")
            return None
        return res

    def move_hand_to_coord(self, x: float, y: float, z: float, frame_id: str = "base_footprint", duration_sec: float = 3.0, use_base_motion: bool = True) -> bool:
        res = self.get_hand_to_coord(x, y, z, frame_id=frame_id)
        if res is None:
            return False
        if use_base_motion:
            # hsrb_library returns an optional base approach pose.  Keep it conservative:
            # rotate first, then move only the forward component.
            yaw = self._yaw_from_quat(res.move_pose.orientation)
            forward = float(res.move_pose.position.x)
            if abs(yaw) > 0.03:
                self.move_wheel_rotate(yaw)
            if abs(forward) > 0.02:
                self.move_wheel_linear(forward)
        return self.move_joint(list(res.target_joint_names), list(res.target_joint_rad), duration_sec=duration_sec)

    def _send_action_goal(self, client: ActionClient, goal, timeout_sec: float, label: str) -> bool:
        self.node.get_logger().info(f"[HSRB] sending {label}")
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        goal_handle = future.result()
        if goal_handle is None:
            self.node.get_logger().error(f"[HSRB] {label}: goal handle is None")
            return False
        if not goal_handle.accepted:
            self.node.get_logger().error(f"[HSRB] {label}: rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
        result_wrap = result_future.result()
        if result_wrap is None:
            self.node.get_logger().error(f"[HSRB] {label}: result timeout")
            return False
        result = result_wrap.result
        success = bool(getattr(result, "success", True))
        message = getattr(result, "message", "")
        if success:
            self.node.get_logger().info(f"[HSRB] {label}: succeeded {message}")
        else:
            self.node.get_logger().error(f"[HSRB] {label}: failed {message}")
        return success

    @staticmethod
    def _yaw_from_quat(q) -> float:
        import math
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))