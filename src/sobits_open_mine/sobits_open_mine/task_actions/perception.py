from __future__ import annotations

import math
import statistics
import struct
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray

from ..utils import bb_get, bb_set

# 検出した物体のラベルを取得
def get_detection_label(detection: Detection2D) -> str:
    if getattr(detection, "id", ""):
        return str(detection.id)

    if detection.results:
        return str(detection.results[0].hypothesis.class_id)

    return ""

# 検出した物体のスコアを取得
def get_detection_score(detection: Detection2D) -> float:
    if detection.results:
        return float(detection.results[0].hypothesis.score)

    if getattr(detection, "id", ""):
        return 1.0

    return 0.0

# 検出したbboxの中心の点を取得
def get_bbox_center_xy(detection: Detection2D) -> tuple[float, float]:
    """Absorb vision_msgs version differences and return bbox center pixel."""
    center = detection.bbox.center

    if hasattr(center, "position"):
        return float(center.position.x), float(center.position.y)

    return float(center.x), float(center.y)

# bboxのx軸の中心点
def get_bbox_center_x(detection: Detection2D) -> float:
    return get_bbox_center_xy(detection)[0]

# bbox_sizeの取得
def get_bbox_size(detection: Detection2D) -> tuple[float, float]:
    return float(detection.bbox.size_x), float(detection.bbox.size_y)

# ラベルの一致確認
def label_matches(label: str, targets: list[str]) -> bool:
    label_norm = label.strip().lower()
    return any(label_norm == target.strip().lower() for target in targets)


def find_target_detection(
    detections_msg: Detection2DArray | None,
    target_class: str = "person",
    min_score: float = 0.3,
    target_labels: list[str] | None = None,
) -> Detection2D | None:
    if detections_msg is None:
        return None

    targets = target_labels if target_labels else [target_class]
    best_detection: Detection2D | None = None
    best_score = -1.0

    for detection in detections_msg.detections:
        label = get_detection_label(detection)
        score = get_detection_score(detection)

        if not label_matches(label, targets):
            continue

        if score < min_score:
            continue

        if score > best_score:
            best_score = score
            best_detection = detection

    return best_detection


def detection_to_dict(detection: Detection2D) -> dict[str, Any]:
    cx, cy = get_bbox_center_xy(detection)
    sx, sy = get_bbox_size(detection)
    return {
        "id": get_detection_label(detection),
        "score": get_detection_score(detection),
        "bbox_center_x": cx,
        "bbox_center_y": cy,
        "bbox_size_x": sx,
        "bbox_size_y": sy,
    }

# 人検出後に人のいる方に回転
def turn_to_human(node, blackboard: Any) -> str:
    params = bb_get(blackboard, "task_params", {}).get("human_detection", {})

    target_class = str(params.get("target_class", "person"))
    detection_timeout_sec = float(params.get("detection_timeout_sec", 15.0))
    center_tolerance_px = float(params.get("center_tolerance_px", 60.0))
    turn_speed = float(params.get("turn_speed", 0.25))
    image_width = float(params.get("image_width", 640.0))
    min_score = float(params.get("min_score", 0.3))

    # 画像座標のx軸中心
    image_center_x = image_width / 2.0

    node.get_logger().info("[HUMAN] start human detection")
    node.say("人を探します")

    start_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        human_detections = getattr(node, "latest_human_detections", None)
        if human_detections is None:
            human_detections = node.latest_detections

        detection = find_target_detection(
            human_detections,
            target_class=target_class,
            min_score=min_score,
        )

        if detection is None:
            node.publish_cmd_vel(0.0, 0.0)

            if time.time() - start_time > detection_timeout_sec:
                node.get_logger().warn("[HUMAN] human detection timeout. retry")
                return "retry"

            continue

        bbox_center_x = get_bbox_center_x(detection)
        error_x = bbox_center_x - image_center_x

        node.get_logger().info(
            f"[HUMAN] found {target_class}: x={bbox_center_x:.1f}, error={error_x:.1f}"
        )

        if abs(error_x) <= center_tolerance_px:
            node.stop_robot()
            node.get_logger().info("[HUMAN] human is centered")
            node.say("見つけました")
            bb_set(blackboard, "human_detected", True)
            return "succeeded"

        angular_z = -turn_speed if error_x > 0 else turn_speed
        node.publish_cmd_vel(0.0, angular_z)

        if time.time() - start_time > detection_timeout_sec:
            node.stop_robot()
            node.get_logger().warn("[HUMAN] turn to human timeout. retry")
            return "retry"

    node.stop_robot()
    return "aborted"

def look_down_for_object(node, blackboard: Any) -> None:
    params = bb_get(blackboard, "task_params", {}).get("object_detection", {})

    head_pan = float(params.get("object_head_pan_joint_rad", 0.0))
    head_tilt = float(params.get("object_head_tilt_joint_rad", -0.5))
    duration = float(params.get("object_head_duration_sec", 1.5))

    node.get_logger().info(
        f"[OBJECT] look down: head_pan_joint={head_pan:.3f}, head_tilt_joint={head_tilt:.3f}"
    )

    node.hsrb.move_joint(
        ["head_pan_joint", "head_tilt_joint"],
        [head_pan, head_tilt],
        duration_sec=duration,
    )

def detect_target_object(node, blackboard: Any) -> str:
    """Find the object requested in LISTEN_COMMAND from YOLO detections."""
    params = bb_get(blackboard, "task_params", {}).get("object_detection", {})

    detection_timeout_sec = float(params.get("detection_timeout_sec", 15.0))
    min_score = float(params.get("min_score", 0.2))
    retry_when_not_found = bool(params.get("retry_when_not_found", True))

    target_object = str(bb_get(blackboard, "target_object", ""))
    target_labels = bb_get(blackboard, "yolo_target_labels", [])
    if not isinstance(target_labels, list):
        target_labels = [str(target_labels)]

    if not target_object:
        node.get_logger().error("[OBJECT] target_object is empty. Did LISTEN_COMMAND parse an object?")
        return "aborted"

    if not target_labels:
        target_labels = [target_object]

    node.get_logger().info(f"[OBJECT] target_object={target_object}, labels={target_labels}")
    node.say(f"{target_object}を探します")

    look_down_for_object(node, blackboard)

    start_time = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        detection = find_target_detection(
            node.latest_detections,
            target_class=target_object,
            min_score=min_score,
            target_labels=target_labels,
        )

        if detection is not None:
            detection_dict = detection_to_dict(detection)
            bb_set(blackboard, "target_detection", detection_dict)
            node.get_logger().info(f"[OBJECT] found: {detection_dict}")
            return "succeeded"

        if time.time() - start_time > detection_timeout_sec:
            node.get_logger().warn("[OBJECT] object detection timeout")
            return "retry" if retry_when_not_found else "aborted"

    return "aborted"


def _read_depth_value_m(depth_image: Image, u: int, v: int) -> float | None:
    if u < 0 or v < 0 or u >= depth_image.width or v >= depth_image.height:
        return None

    offset = v * depth_image.step + u * (depth_image.step // depth_image.width)
    enc = depth_image.encoding.upper()

    try:
        if enc in {"16UC1", "MONO16"}:
            raw = depth_image.data[offset : offset + 2]
            if len(raw) < 2:
                return None
            value_mm = struct.unpack_from("<H", raw)[0]
            if value_mm == 0:
                return None
            return float(value_mm) / 1000.0

        if enc in {"32FC1"}:
            raw = depth_image.data[offset : offset + 4]
            if len(raw) < 4:
                return None
            value_m = struct.unpack_from("<f", raw)[0]
            if not math.isfinite(value_m) or value_m <= 0.0:
                return None
            return float(value_m)
    except Exception:
        return None

    return None


def _median_depth_around(depth_image: Image, u: int, v: int, window_px: int = 7) -> float | None:
    radius = max(0, int(window_px) // 2)
    values: list[float] = []

    for yy in range(v - radius, v + radius + 1):
        for xx in range(u - radius, u + radius + 1):
            value = _read_depth_value_m(depth_image, xx, yy)
            if value is not None:
                values.append(value)

    if not values:
        return None

    return float(statistics.median(values))


def estimate_object_pose_from_depth(node, blackboard: Any) -> str:
    """Estimate target object 3D position in the RGB-D camera frame.

    This is the first grasp-ready scaffold. It stores the 3D point in blackboard;
    actual arm IK / MoveIt grasp execution can consume this later.
    """
    params = bb_get(blackboard, "task_params", {}).get("pose_estimation", {})
    depth_window_px = int(params.get("depth_window_px", 7))
    min_depth_m = float(params.get("min_depth_m", 0.15))
    max_depth_m = float(params.get("max_depth_m", 2.5))

    detection = bb_get(blackboard, "target_detection", {})
    if not detection:
        node.get_logger().error("[POSE] target_detection is empty")
        return "aborted"

    depth_image = node.latest_depth_image
    camera_info = node.latest_camera_info
    if depth_image is None:
        node.get_logger().error("[POSE] depth image is not available")
        return "retry"

    if camera_info is None:
        node.get_logger().error("[POSE] camera_info is not available")
        return "retry"

    u = int(round(float(detection["bbox_center_x"])))
    v = int(round(float(detection["bbox_center_y"])))
    z = _median_depth_around(depth_image, u, v, window_px=depth_window_px)

    if z is None:
        node.get_logger().warn("[POSE] valid depth not found around bbox center")
        return "retry"

    if not (min_depth_m <= z <= max_depth_m):
        node.get_logger().warn(f"[POSE] depth out of range: {z:.3f} m")
        return "retry"

    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])

    if fx == 0.0 or fy == 0.0:
        node.get_logger().error("[POSE] invalid camera_info intrinsics")
        return "aborted"

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    pose_camera = {
        "frame_id": camera_info.header.frame_id,
        "x": x,
        "y": y,
        "z": z,
        "pixel_u": u,
        "pixel_v": v,
    }
    bb_set(blackboard, "target_pose_camera", pose_camera)

    node.get_logger().info(f"[POSE] target_pose_camera: {pose_camera}")

    # For grasping, hsrb_library expects target coordinates in base_footprint.
    # Convert camera point to the node's base_frame when TF is available.
    pose_base = None
    if hasattr(node, "transform_point_to_base"):
        pose_base = node.transform_point_to_base(camera_info.header.frame_id, x, y, z)

    if pose_base is not None:
        pose_base["pixel_u"] = u
        pose_base["pixel_v"] = v
        pose_base["source_frame_id"] = camera_info.header.frame_id
        bb_set(blackboard, "target_pose_base", pose_base)
        node.get_logger().info(f"[POSE] target_pose_base: {pose_base}")
        return "succeeded"

    # Keep target_pose_camera for debugging, but grasp should not use it directly.
    node.get_logger().warn("[POSE] target_pose_base could not be generated. TF may not be ready.")
    return "retry"
