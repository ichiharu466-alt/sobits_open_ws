from __future__ import annotations
from ..utils import bb_get

import math
import time
from typing import Any

import rclpy
from sensor_msgs.msg import LaserScan


# Lidarの点情報を前方に絞る関数　
# 引数
def get_front_min_distance(
    scan: LaserScan | None,
    angle_width_deg: float = 15.0,
) -> float | None:
    if scan is None:
        return None

    angle_width = math.radians(angle_width_deg)
    distances: list[float] = []

    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment

        if -angle_width <= angle <= angle_width:
            if math.isfinite(r):
                if scan.range_min <= r <= scan.range_max:
                    distances.append(r)

    if not distances:
        return None

    return min(distances)

# ドアが閉まっているを判断する関数
# 引数
def is_door_closed(
    node,
    angle_width_deg: float = 15.0,
    closed_distance_threshold: float = 0.8,
) -> bool:
    front_dist = get_front_min_distance(
        node.latest_scan,
        angle_width_deg=angle_width_deg,
    )

    if front_dist is None:
        node.get_logger().warn(
            "[DOOR] No valid front LaserScan. Treat as door closed."
        )
        return True

    node.get_logger().info(f"[DOOR] front distance: {front_dist:.2f} m")

    return front_dist < closed_distance_threshold

# 今までの関数 + 待つ動作を加えた最終的なdoor_open関数
# 引数
def wait_for_door_open(node, blackboard: Any) -> str:
    params = bb_get(blackboard, "task_params", {}).get("door", {})

    angle_width_deg = float(params.get("front_angle_width_deg", 15.0))
    closed_distance_threshold = float(params.get("closed_distance_threshold", 0.8))
    wait_timeout_sec = float(params.get("wait_timeout_sec", 10.0))
    request_interval_sec = float(params.get("request_interval_sec", 3.0))

    node.get_logger().info("[DOOR] start door open check")
    node.say("ドアを開けてください")

    start_time = time.time()
    last_request_time = start_time

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        closed = is_door_closed(
            node,
            angle_width_deg=angle_width_deg,
            closed_distance_threshold=closed_distance_threshold,
        )

        if not closed:
            node.get_logger().info("[DOOR] door is open")
            node.say("ありがとうございます")
            return "succeeded"

        now = time.time()

        if now - last_request_time > request_interval_sec:
            node.say("ドアを開けてください")
            last_request_time = now

        if now - start_time > wait_timeout_sec:
            node.get_logger().warn("[DOOR] timeout. retry")
            return "retry"

    return "aborted"
