from __future__ import annotations

import math
import time
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ..utils import bb_get, bb_set
from .perception import detect_target_object, estimate_object_pose_from_depth


def _cfg(blackboard: Any) -> dict[str, Any]:
    return bb_get(blackboard, "task_params", {}).get("grasp", {})


def _to_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"true", "1", "yes", "on"}
    return bool(value)


def _set_failure(blackboard: Any, stage: str, message: str = "") -> bool:
    bb_set(blackboard, "grasp_failure_stage", stage)
    bb_set(blackboard, "grasp_failure_message", message)
    return False


def _move_pose(node: Any, pose_name: str, duration_sec: float) -> bool:
    if not pose_name:
        return True

    node.get_logger().info(f"[GRASP] move_to_pose: {pose_name}")
    ok = node.hsrb.move_to_pose(pose_name, duration_sec=duration_sec)

    if not ok:
        node.get_logger().error(f"[GRASP] move_to_pose failed: {pose_name}")

    return bool(ok)


def _move_joint(
    node: Any,
    joints: list[str],
    values: list[float],
    duration_sec: float,
) -> bool:
    if len(joints) != len(values):
        node.get_logger().error(
            f"[GRASP] move_joint size mismatch: joints={len(joints)}, values={len(values)}"
        )
        return False

    command = dict(zip(joints, values))
    node.get_logger().info(f"[GRASP] move_joint: {command}")

    ok = node.hsrb.move_joint(joints, values, duration_sec=duration_sec)

    if not ok:
        node.get_logger().error(f"[GRASP] move_joint failed: {command}")

    return bool(ok)


def _sec_to_duration(sec: float) -> Duration:
    sec = max(0.0, float(sec))
    duration = Duration()
    duration.sec = int(sec)
    duration.nanosec = int((sec - int(sec)) * 1_000_000_000)
    return duration


def _publish_gripper_trajectory(
    node: Any,
    joint: str,
    rad: float,
    duration_sec: float,
) -> bool:
    publisher = getattr(node, "_gripper_traj_pub", None)

    if publisher is None:
        publisher = node.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )
        setattr(node, "_gripper_traj_pub", publisher)
        time.sleep(0.2)

    message = JointTrajectory()
    message.header.stamp = node.get_clock().now().to_msg()
    message.joint_names = [joint]

    point = JointTrajectoryPoint()
    point.positions = [float(rad)]
    point.time_from_start = _sec_to_duration(duration_sec)
    message.points = [point]

    node.get_logger().info(
        f"[GRASP] gripper: {joint}={rad:.3f} rad, duration={duration_sec:.2f}s"
    )
    publisher.publish(message)
    return True


def _open_gripper(node: Any, cfg: dict[str, Any]) -> bool:
    joint = str(cfg.get("gripper_joint", "hand_motor_joint"))
    rad = float(cfg.get("gripper_open_rad", 1.0))
    duration = float(cfg.get("gripper_duration_sec", 1.0))
    wait = float(cfg.get("gripper_open_wait_sec", duration + 0.3))

    ok = _publish_gripper_trajectory(node, joint, rad, duration)
    time.sleep(max(0.0, wait))
    return ok


def _close_gripper(node: Any, cfg: dict[str, Any]) -> bool:
    joint = str(cfg.get("gripper_joint", "hand_motor_joint"))
    rad = float(cfg.get("gripper_close_rad", 0.3))
    duration = float(cfg.get("gripper_duration_sec", 1.0))
    wait = float(cfg.get("gripper_close_wait_sec", duration + 0.6))

    ok = _publish_gripper_trajectory(node, joint, rad, duration)
    time.sleep(max(0.0, wait))
    return ok



def _ensure_effort_subscription(node: Any) -> None:
    """把持判定用に/joint_statesのeffortを保存する。"""
    if hasattr(node, "_grasp_effort_sub"):
        return

    setattr(node, "_grasp_joint_efforts", {})

    def callback(message: JointState) -> None:
        efforts = getattr(node, "_grasp_joint_efforts", {})
        for index, name in enumerate(message.name):
            if index < len(message.effort):
                efforts[name] = float(message.effort[index])
        setattr(node, "_grasp_joint_efforts", efforts)

    subscription = node.create_subscription(JointState, "/joint_states", callback, 10)
    setattr(node, "_grasp_effort_sub", subscription)


def _is_object_grasped(node: Any, cfg: dict[str, Any]) -> bool:
    efforts = getattr(node, "_grasp_joint_efforts", {})
    right_joint = str(
        cfg.get("grasp_right_effort_joint", "hand_r_spring_proximal_joint")
    )
    left_joint = str(
        cfg.get("grasp_left_effort_joint", "hand_l_spring_proximal_joint")
    )
    threshold = float(cfg.get("grasp_effort_threshold", 2.0))

    right = efforts.get(right_joint)
    left = efforts.get(left_joint)
    if right is None or left is None:
        return False

    node.get_logger().info(
        f"[GRASP] finger effort: right={right:.3f}, left={left:.3f}, "
        f"threshold={threshold:.3f}"
    )
    return right >= threshold and left >= threshold


def _wait_for_grasp_confirmation(node: Any, cfg: dict[str, Any]) -> bool:
    """指のeffortが閾値を超えるまで短時間確認する。"""
    _ensure_effort_subscription(node)
    timeout = float(cfg.get("grasp_check_timeout_sec", 3.0))
    required_count = int(cfg.get("grasp_required_consecutive_samples", 3))
    consecutive = 0
    start = time.monotonic()

    while rclpy.ok() and time.monotonic() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if _is_object_grasped(node, cfg):
            consecutive += 1
            if consecutive >= required_count:
                node.get_logger().info("[GRASP] grasp confirmed by finger effort")
                return True
        else:
            consecutive = 0

    node.get_logger().warn("[GRASP] grasp was not confirmed by finger effort")
    return False


def _get_omni_publisher(node: Any):
    publisher = getattr(node, "_grasp_omni_cmd_pub", None)
    if publisher is not None:
        return publisher

    topic = "/cmd_vel"
    try:
        topic = str(node.get_parameter("cmd_vel_topic").value)
    except Exception:
        pass

    publisher = node.create_publisher(Twist, topic, 10)
    setattr(node, "_grasp_omni_cmd_pub", publisher)
    node.get_logger().info(f"[GRASP] omni cmd_vel topic: {topic}")
    return publisher


def _publish_omni_velocity(node: Any, linear_x: float, linear_y: float) -> None:
    message = Twist()
    message.linear.x = float(linear_x)
    message.linear.y = float(linear_y)
    _get_omni_publisher(node).publish(message)


def _stop_omni(node: Any) -> None:
    _publish_omni_velocity(node, 0.0, 0.0)


def _get_arm_center_y(node: Any, cfg: dict[str, Any]) -> float | None:
    """腕基準TFのbase_footprint座標系におけるy座標を取得する。"""
    base_frame = str(cfg.get("arm_center_base_frame", "base_footprint"))
    arm_frame = str(cfg.get("arm_center_frame", "hand_palm_link"))
    timeout_sec = float(cfg.get("arm_center_tf_timeout_sec", 2.0))

    tf_buffer = getattr(node, "tf_buffer", None)
    if tf_buffer is None:
        node.get_logger().error("[GRASP] node.tf_buffer is not available")
        return None

    deadline = time.monotonic() + timeout_sec
    last_error = ""

    while rclpy.ok() and time.monotonic() < deadline:
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                arm_frame,
                rclpy.time.Time(),
            )
            arm_y = float(transform.transform.translation.y)
            node.get_logger().info(
                f"[GRASP] arm center TF: {base_frame} -> {arm_frame}, "
                f"y={arm_y:.3f} m"
            )
            return arm_y
        except Exception as exc:
            last_error = str(exc)
            rclpy.spin_once(node, timeout_sec=0.05)

    fallback = cfg.get("arm_center_y_fallback_m", None)
    if fallback is not None:
        arm_y = float(fallback)
        node.get_logger().warn(
            f"[GRASP] arm center TF unavailable; using fallback y={arm_y:.3f} m. "
            f"error={last_error}"
        )
        return arm_y

    node.get_logger().error(
        f"[GRASP] failed to get arm center TF: "
        f"{base_frame} -> {arm_frame}: {last_error}"
    )
    return None


def _move_laterally_once(
    node: Any,
    distance_y_m: float,
    cfg: dict[str, Any],
) -> bool:
    """時間制御cmd_velで一度だけ左右移動する。"""
    max_distance = float(cfg.get("max_lateral_alignment_m", 0.40))
    speed = abs(float(cfg.get("lateral_alignment_speed_mps", 0.05)))

    if abs(distance_y_m) > max_distance:
        node.get_logger().error(
            f"[GRASP] lateral movement too large: "
            f"{distance_y_m:.3f} m, max={max_distance:.3f} m"
        )
        return False

    if speed <= 0.0:
        node.get_logger().error(
            "[GRASP] lateral_alignment_speed_mps must be positive"
        )
        return False

    direction_sign = float(cfg.get("lateral_direction_sign", 1.0))
    lateral_speed = direction_sign * (
        speed if distance_y_m > 0.0 else -speed
    )
    duration = abs(distance_y_m) / speed

    node.get_logger().info(
        f"[GRASP] lateral move: distance={distance_y_m:.3f} m, "
        f"speed={lateral_speed:.3f} m/s, duration={duration:.2f}s"
    )

    start = time.monotonic()

    try:
        while rclpy.ok() and time.monotonic() - start < duration:
            _publish_omni_velocity(node, 0.0, lateral_speed)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.04)
    finally:
        _stop_omni(node)

    time.sleep(0.3)
    return True


def _align_target_to_arm(
    node: Any,
    blackboard: Any,
    pose_base: dict[str, Any],
    cfg: dict[str, Any],
) -> dict[str, Any] | None:
    """物体のy座標を腕基準TFのy座標へ合わせる。"""
    arm_y = _get_arm_center_y(node, cfg)
    if arm_y is None:
        return None

    target_y = float(pose_base.get("y", 0.0))
    y_error = target_y - arm_y

    tolerance = float(cfg.get("lateral_alignment_tolerance_m", 0.04))
    result_tolerance = float(
        cfg.get("lateral_alignment_result_tolerance_m", 0.08)
    )

    node.get_logger().info(
        f"[GRASP] arm alignment: target_y={target_y:.3f}, "
        f"arm_y={arm_y:.3f}, error={y_error:.3f} m"
    )

    if abs(y_error) <= tolerance:
        node.get_logger().info("[GRASP] target is aligned with arm TF")
        return pose_base

    if not _move_laterally_once(node, y_error, cfg):
        return None

    refreshed_pose = _refresh_target_pose(node, blackboard, cfg)
    if refreshed_pose is None:
        return None

    refreshed_arm_y = _get_arm_center_y(node, cfg)
    if refreshed_arm_y is None:
        return None

    final_target_y = float(refreshed_pose.get("y", 0.0))
    final_error = final_target_y - refreshed_arm_y

    node.get_logger().info(
        f"[GRASP] arm alignment result: target_y={final_target_y:.3f}, "
        f"arm_y={refreshed_arm_y:.3f}, error={final_error:.3f} m"
    )

    if abs(final_error) > result_tolerance:
        node.get_logger().error(
            f"[GRASP] arm alignment insufficient: "
            f"error={final_error:.3f} m, tolerance={result_tolerance:.3f} m"
        )
        return None

    return refreshed_pose

def _safe_check(node: Any, pose_base: dict[str, Any], cfg: dict[str, Any]) -> bool:
    x = float(pose_base.get("x", 0.0))
    y = float(pose_base.get("y", 0.0))
    z = float(pose_base.get("z", 0.0))

    min_x = float(cfg.get("min_x_m", 0.20))
    max_x = float(cfg.get("max_x_m", 1.30))
    max_abs_y = float(cfg.get("max_abs_y_m", 0.65))
    min_z = float(cfg.get("min_z_m", 0.03))
    max_z = float(cfg.get("max_z_m", 1.20))

    if not min_x <= x <= max_x:
        node.get_logger().error(
            f"[GRASP] target x out of range: {x:.3f}, "
            f"allowed=[{min_x:.3f}, {max_x:.3f}]"
        )
        return False

    if abs(y) > max_abs_y:
        node.get_logger().error(
            f"[GRASP] target y too far: {y:.3f}, max_abs_y={max_abs_y:.3f}"
        )
        return False

    if not min_z <= z <= max_z:
        node.get_logger().error(
            f"[GRASP] target z out of range: {z:.3f}, "
            f"allowed=[{min_z:.3f}, {max_z:.3f}]"
        )
        return False

    return True


def _refresh_target_pose(
    node: Any,
    blackboard: Any,
    cfg: dict[str, Any],
) -> dict[str, Any] | None:
    node.latest_detections = None
    node.latest_depth_image = None
    bb_set(blackboard, "target_detection", {})
    bb_set(blackboard, "target_pose_camera", {})
    bb_set(blackboard, "target_pose_base", {})

    time.sleep(max(0.0, float(cfg.get("refresh_pose_wait_sec", 0.5))))

    if detect_target_object(node, blackboard) != "succeeded":
        node.get_logger().error("[GRASP] target re-detection failed")
        return None

    if estimate_object_pose_from_depth(node, blackboard) != "succeeded":
        node.get_logger().error("[GRASP] target pose refresh failed")
        return None

    pose = bb_get(blackboard, "target_pose_base", {})
    return pose if pose else None


def _rotate_with_cmd_vel(
    node: Any,
    yaw_rad: float,
    cfg: dict[str, Any],
) -> bool:
    max_yaw = float(cfg.get("max_alignment_yaw_rad", 0.80))
    speed = abs(float(cfg.get("alignment_angular_speed_rps", 0.20)))

    if abs(yaw_rad) > max_yaw:
        node.get_logger().error(
            f"[GRASP] alignment yaw too large: {yaw_rad:.3f} rad, max={max_yaw:.3f}"
        )
        return False

    if speed <= 0.0:
        node.get_logger().error("[GRASP] alignment_angular_speed_rps must be positive")
        return False

    direction = 1.0 if yaw_rad >= 0.0 else -1.0
    duration = abs(yaw_rad) / speed
    period = 0.05

    node.get_logger().warn(
        f"[GRASP] cmd_vel rotation fallback: yaw={yaw_rad:.3f}, "
        f"speed={direction * speed:.3f}, duration={duration:.2f}s"
    )

    start = time.monotonic()

    try:
        while rclpy.ok() and time.monotonic() - start < duration:
            node.publish_cmd_vel(0.0, direction * speed)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period)
    finally:
        node.stop_robot()

    time.sleep(0.3)
    return True


def _face_target(
    node: Any,
    blackboard: Any,
    pose_base: dict[str, Any],
    cfg: dict[str, Any],
) -> dict[str, Any] | None:
    x = float(pose_base.get("x", 0.0))
    y = float(pose_base.get("y", 0.0))
    yaw = math.atan2(y, x)
    tolerance = float(cfg.get("alignment_yaw_tolerance_rad", 0.05))

    node.get_logger().info(
        f"[GRASP] target alignment: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad"
    )

    if abs(yaw) <= tolerance:
        node.get_logger().info("[GRASP] target is already centered")
        return pose_base

    rotate_timeout = float(cfg.get("rotate_action_timeout_sec", 15.0))
    ok = node.hsrb.move_wheel_rotate(yaw, timeout_sec=rotate_timeout)

    refreshed = _refresh_target_pose(node, blackboard, cfg)
    if refreshed is None:
        return None

    remaining_yaw = math.atan2(
        float(refreshed.get("y", 0.0)),
        float(refreshed.get("x", 0.0)),
    )

    if not ok and abs(remaining_yaw) > tolerance:
        node.get_logger().warn(
            f"[GRASP] wheel rotation returned False; remaining_yaw={remaining_yaw:.3f}"
        )

        if not _to_bool(cfg.get("use_cmd_vel_rotation_fallback", True)):
            return None

        if not _rotate_with_cmd_vel(node, remaining_yaw, cfg):
            return None

        refreshed = _refresh_target_pose(node, blackboard, cfg)
        if refreshed is None:
            return None

        remaining_yaw = math.atan2(
            float(refreshed.get("y", 0.0)),
            float(refreshed.get("x", 0.0)),
        )

    node.get_logger().info(
        f"[GRASP] alignment result: remaining_yaw={remaining_yaw:.3f} rad"
    )

    max_remaining = float(cfg.get("alignment_result_tolerance_rad", 0.12))
    if abs(remaining_yaw) > max_remaining:
        node.get_logger().error(
            f"[GRASP] target still not centered: {remaining_yaw:.3f} rad"
        )
        return None

    return refreshed

# 物体との距離による前進
def _move_forward_with_cmd_vel(
    node: Any,
    distance_m: float,
    cfg: dict[str, Any],
) -> bool:
    max_distance = float(cfg.get("max_cmd_vel_approach_m", 0.40))
    speed = abs(float(cfg.get("approach_linear_speed_mps", 0.06)))

    if abs(distance_m) > max_distance:
        node.get_logger().error(
            f"[GRASP] cmd_vel approach too large: {distance_m:.3f} m, "
            f"max={max_distance:.3f} m"
        )
        return False

    if speed <= 0.0:
        node.get_logger().error("[GRASP] approach_linear_speed_mps must be positive")
        return False

    direction = 1.0 if distance_m >= 0.0 else -1.0
    duration = abs(distance_m) / speed
    period = 0.05

    node.get_logger().warn(
        f"[GRASP] cmd_vel approach fallback: distance={distance_m:.3f}, "
        f"speed={direction * speed:.3f}, duration={duration:.2f}s"
    )

    start = time.monotonic()

    try:
        while rclpy.ok() and time.monotonic() - start < duration:
            node.publish_cmd_vel(direction * speed, 0.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(period)
    finally:
        node.stop_robot()

    time.sleep(0.3)
    return True


def _approach_target(
    node: Any,
    blackboard: Any,
    pose_base: dict[str, Any],
    cfg: dict[str, Any],
) -> dict[str, Any] | None:
    target_x = float(pose_base.get("x", 0.0))
    desired_x = float(cfg.get("desired_hand_target_x_m", 0.63))
    deadband = float(cfg.get("approach_deadband_m", 0.03))
    distance = target_x - desired_x

    if distance < 0.0 and not _to_bool(cfg.get("allow_backward_approach", False)):
        distance = 0.0

    if abs(distance) <= deadband:
        node.get_logger().info("[GRASP] base approach is not required")
        return pose_base

    max_total = float(cfg.get("max_total_approach_m", 0.70))
    if abs(distance) > max_total:
        node.get_logger().error(
            f"[GRASP] approach distance too large: {distance:.3f} m, "
            f"max={max_total:.3f} m"
        )
        return None

    timeout = float(cfg.get("wheel_action_timeout_sec", 20.0))
    node.get_logger().info(
        f"[GRASP] base approach: target_x={target_x:.3f}, "
        f"desired_x={desired_x:.3f}, move={distance:.3f} m"
    )

    ok = node.hsrb.move_wheel_linear(distance, timeout_sec=timeout)

    refreshed = _refresh_target_pose(node, blackboard, cfg)
    if refreshed is None:
        return None

    final_x = float(refreshed.get("x", 0.0))
    remaining_distance = final_x - desired_x
    max_error = float(cfg.get("approach_result_tolerance_m", 0.15))

    if not ok and abs(remaining_distance) > max_error:
        node.get_logger().warn(
            f"[GRASP] wheel Action returned False; "
            f"remaining_distance={remaining_distance:.3f} m"
        )

        if not _to_bool(cfg.get("use_cmd_vel_approach_fallback", True)):
            return None

        if remaining_distance < 0.0 and not _to_bool(
            cfg.get("allow_backward_approach", False)
        ):
            node.get_logger().error(
                "[GRASP] robot is already too close; backward fallback is disabled"
            )
            return None

        if not _move_forward_with_cmd_vel(node, remaining_distance, cfg):
            return None

        refreshed = _refresh_target_pose(node, blackboard, cfg)
        if refreshed is None:
            return None

        final_x = float(refreshed.get("x", 0.0))

    node.get_logger().info(
        f"[GRASP] approach result: target_x={final_x:.3f}, desired_x={desired_x:.3f}"
    )

    if abs(final_x - desired_x) > max_error:
        node.get_logger().error(
            f"[GRASP] approach result outside tolerance: error={final_x - desired_x:.3f} m"
        )
        return None

    return refreshed

# armを指定位置に戻す
def _return_arm_home(node: Any, cfg: dict[str, Any]) -> bool:
    arm_home_joints = [
        "arm_flex_joint",
        "arm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    ]
    arm_home_values = [
        float(cfg.get("home_arm_flex_joint_rad", 0.0)),
        float(cfg.get("home_arm_roll_joint_rad", 0.0)),
        float(cfg.get("carry_wrist_flex_joint_rad", -1.57)),
        float(cfg.get("home_wrist_roll_joint_rad", 0.0)),
    ]

    if not _move_joint(
        node,
        arm_home_joints,
        arm_home_values,
        duration_sec=float(cfg.get("return_arm_duration_sec", 3.0)),
    ):
        return False

    return _move_joint(
        node,
        ["arm_lift_joint"],
        [float(cfg.get("home_arm_lift_joint_rad", 0.0))],
        duration_sec=float(cfg.get("return_lift_duration_sec", 2.0)),
    )


def _hsrb_library_grasp(
    node: Any,
    blackboard: Any,
    pose_base: dict[str, Any],
) -> bool:
    cfg = _cfg(blackboard)
    bb_set(blackboard, "grasp_failure_stage", "")
    bb_set(blackboard, "grasp_failure_message", "")

    aligned_pose = _align_target_to_arm(node, blackboard, pose_base, cfg)
    if aligned_pose is None:
        return _set_failure(blackboard, "arm_tf_lateral_alignment")

    approached_pose = _approach_target(node, blackboard, aligned_pose, cfg)
    if approached_pose is None:
        return _set_failure(blackboard, "approach_target")

    if not _safe_check(node, approached_pose, cfg):
        return _set_failure(blackboard, "final_safety_check")

    frame_id = str(approached_pose.get("frame_id", "base_footprint"))
    target_x = float(approached_pose.get("x", 0.0)) + float(
        cfg.get("target_offset_x_m", 0.07)
    )
    target_y = float(approached_pose.get("y", 0.0)) + float(
        cfg.get("target_offset_y_m", 0.0)
    )
    target_z = float(approached_pose.get("z", 0.0)) + float(
        cfg.get("target_offset_z_m", 0.00)
    )

    node.get_logger().info(
        f"[GRASP] final target: frame={frame_id}, "
        f"xyz=({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
    )

    if not _open_gripper(node, cfg):
        return _set_failure(blackboard, "open_gripper")

    ready_pose = str(cfg.get("ready_pose", ""))
    motion_duration = float(cfg.get("motion_duration_sec", 3.0))

    if ready_pose and not _move_pose(node, ready_pose, motion_duration):
        return _set_failure(blackboard, "ready_pose", ready_pose)

    if not node.hsrb.move_hand_to_coord(
        target_x,
        target_y,
        target_z,
        frame_id=frame_id,
        duration_sec=motion_duration,
        use_base_motion=False,
    ):
        return _set_failure(
            blackboard,
            "move_hand_to_coord",
            f"x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}",
        )

    _ensure_effort_subscription(node)
    if not _close_gripper(node, cfg):
        return _set_failure(blackboard, "close_gripper")

    if not _wait_for_grasp_confirmation(node, cfg):
        if _to_bool(cfg.get("open_gripper_when_grasp_not_confirmed", True)):
            _open_gripper(node, cfg)
        return _set_failure(blackboard, "grasp_confirmation")

    lift_joint = str(cfg.get("lift_joint", "arm_lift_joint"))
    lift_rad = float(cfg.get("lift_joint_rad", 0.35))
    lift_duration = float(cfg.get("lift_duration_sec", 1.5))

    if not _move_joint(
        node,
        [lift_joint],
        [lift_rad],
        duration_sec=lift_duration,
    ):
        return _set_failure(blackboard, "lift_after_grasp")

    if _to_bool(cfg.get("return_arm_home_after_grasp", True)):
        if not _return_arm_home(node, cfg):
            return _set_failure(blackboard, "return_arm_home")

    return True


def grasp_object(node: Any, blackboard: Any) -> str:
    target_object = bb_get(blackboard, "target_object", "object")
    initial_camera = bb_get(blackboard, "target_pose_camera", {})
    initial_base = bb_get(blackboard, "target_pose_base", {})

    node.get_logger().info("========== GRASP_OBJECT ==========")
    node.get_logger().info(f"[GRASP] target_object={target_object}")
    node.get_logger().info(f"[GRASP] target_pose_camera={initial_camera}")
    node.get_logger().info(f"[GRASP] target_pose_base={initial_base}")

    if not initial_base:
        bb_set(blackboard, "object_grasped", False)
        bb_set(
            blackboard,
            "grasp_plan",
            {
                "target_object": target_object,
                "status": "failed",
                "failure_stage": "no_target_pose_base",
            },
        )
        return "aborted"

    ok = _hsrb_library_grasp(node, blackboard, initial_base)

    final_camera = bb_get(blackboard, "target_pose_camera", initial_camera)
    final_base = bb_get(blackboard, "target_pose_base", initial_base)
    failure_stage = bb_get(blackboard, "grasp_failure_stage", "")
    failure_message = bb_get(blackboard, "grasp_failure_message", "")

    bb_set(blackboard, "object_grasped", bool(ok))
    bb_set(
        blackboard,
        "grasp_plan",
        {
            "target_object": target_object,
            "initial_target_pose_camera": initial_camera,
            "initial_target_pose_base": initial_base,
            "target_pose_camera": final_camera,
            "target_pose_base": final_base,
            "method": "arm_tf_omni_align_approach_and_effort_grasp_check",
            "status": "grasped" if ok else "failed",
            "failure_stage": failure_stage,
            "failure_message": failure_message,
        },
    )

    if ok:
        node.get_logger().info("[GRASP] succeeded")
        return "succeeded"

    node.get_logger().error(
        f"[GRASP] failed: stage={failure_stage or 'unknown'}, "
        f"message={failure_message or '-'}"
    )
    return "aborted"

