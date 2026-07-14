from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def load_yaml(
    path_or_name: str,
    package_name: str | None = None,
    subdir: str = "config",
) -> dict[str, Any]:
    """Load a YAML file.

    If path_or_name is relative and package_name is given, the file is searched
    under share/<package_name>/<subdir>/.
    """
    path = Path(path_or_name)

    if not path.is_absolute() and package_name is not None:
        package_share = Path(get_package_share_directory(package_name))
        path = package_share / subdir / path_or_name

    if not path.exists():
        raise FileNotFoundError(f"YAML file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be dict: {path}")

    return data


def get_locations(locations_yaml: dict[str, Any]) -> dict[str, Any]:
    """Return the actual location dictionary from locations.yaml.

    sobits_open_mine uses the same shape as SOBITS example configs so RViz/TF
    location tools can read it:

        location_pose:
          interaction_point:
            frame_id: map
            translation: ...
            rotation: ...

    State actions, however, want to look up locations by name. This helper makes
    both formats work:
      - nested: locations_yaml["location_pose"][name]
      - flat:   locations_yaml[name]
    """
    if not isinstance(locations_yaml, dict):
        return {}

    nested = locations_yaml.get("location_pose")
    if isinstance(nested, dict):
        return nested

    return locations_yaml


def make_pose_stamped(
    node,
    location: dict[str, Any],
    frame_id: str = "map",
) -> PoseStamped:
    """Convert a location entry into geometry_msgs/PoseStamped.

    Supported formats:
      1. SOBITS location_pose format: frame_id + translation + rotation
      2. Simple format: x + y + yaw
    """
    pose = PoseStamped()
    pose.header.frame_id = str(location.get("frame_id", frame_id))
    pose.header.stamp = node.get_clock().now().to_msg()

    if "translation" in location:
        trans = location.get("translation", {})
        rot = location.get("rotation", {})

        pose.pose.position.x = float(trans.get("x", 0.0))
        pose.pose.position.y = float(trans.get("y", 0.0))
        pose.pose.position.z = float(trans.get("z", 0.0))

        pose.pose.orientation.x = float(rot.get("x", 0.0))
        pose.pose.orientation.y = float(rot.get("y", 0.0))
        pose.pose.orientation.z = float(rot.get("z", 0.0))
        pose.pose.orientation.w = float(rot.get("w", 1.0))
        return pose

    x = float(location.get("x", 0.0))
    y = float(location.get("y", 0.0))
    yaw = float(location.get("yaw", 0.0))

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def bb_get(blackboard, key, default=None):
    """YASMIN Blackboardから安全に値を取得する。

    YASMINのblackboard.get(key)は、keyが存在しないとRuntimeErrorを投げるため、
    存在しない場合はdefaultを返す。
    """
    try:
        value = blackboard.get(key)
    except RuntimeError:
        return default
    except KeyError:
        return default

    return value if value is not None else default


def bb_set(blackboard, key: str, value: Any) -> None:
    """Set a Blackboard value and keep the syntax explicit in task actions."""
    blackboard[key] = value


def normalize_text(text: str) -> str:
    return text.lower().replace("_", " ").replace("-", " ").strip()
