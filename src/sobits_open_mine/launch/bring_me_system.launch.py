from pathlib import Path
from typing import Any

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


PACKAGE_NAME = "sobits_open_mine"


def load_yaml(file_path: Path) -> dict[str, Any]:
    """YAMLファイルを読み込む。"""

    if not file_path.exists():
        raise FileNotFoundError(
            f"YAMLファイルが見つかりません: {file_path}"
        )

    with file_path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(
            f"YAMLの一番上は辞書形式にしてください: {file_path}"
        )

    return data


def to_bool(value: Any) -> bool:
    """YAMLや文字列で指定された値をboolへ変換する。"""

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.lower() in {
            "true",
            "1",
            "yes",
            "on",
        }

    return bool(value)


def require_value(
    config: dict[str, Any],
    key: str,
    config_name: str,
) -> Any:
    """必要な設定値が存在するか確認する。"""

    if key not in config:
        raise ValueError(
            f"{config_name}に'{key}'がありません"
        )

    return config[key]


def create_include_launch(
    launch_name: str,
    launch_config: dict[str, Any],
    additional_arguments: dict[str, Any] | None = None,
) -> IncludeLaunchDescription | None:
    """別パッケージのlaunchファイルを読み込む設定を作る。"""

    if not launch_config:
        raise ValueError(
            f"launch_arg.yamlに'{launch_name}'の設定がありません"
        )

    enabled = to_bool(
        launch_config.get("enabled", True)
    )

    if not enabled:
        return None

    package_name = launch_config.get("package")
    launch_file = launch_config.get("file")

    if not package_name:
        raise ValueError(
            f"{launch_name}: packageが指定されていません"
        )

    if not launch_file:
        raise ValueError(
            f"{launch_name}: fileが指定されていません"
        )

    package_share = Path(
        get_package_share_directory(
            str(package_name)
        )
    )

    launch_path = (
        package_share
        / "launch"
        / str(launch_file)
    )

    if not launch_path.exists():
        raise FileNotFoundError(
            f"{launch_name}のlaunchファイルが見つかりません: "
            f"{launch_path}"
        )

    arguments = dict(
        launch_config.get("arguments", {})
    )

    if additional_arguments:
        arguments.update(additional_arguments)

    # launch_argumentsには文字列として渡す
    string_arguments = {
        str(key): str(value)
        for key, value in arguments.items()
    }

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_path)
        ),
        launch_arguments=string_arguments.items(),
    )


def launch_setup(
    context: LaunchContext,
) -> list:
    """YAMLを読み込み、起動するlaunchとNodeを作成する。"""

    package_share = Path(
        get_package_share_directory(
            PACKAGE_NAME
        )
    )

    config_directory = package_share / "config"

    # launch引数を文字列として取得
    robot_name = LaunchConfiguration(
        "robot"
    ).perform(context)

    skip_profile = LaunchConfiguration(
        "skip_profile"
    ).perform(context)

    # =========================================
    # YAML読み込み
    # =========================================

    robot_yaml = load_yaml(
        config_directory / "robot.yaml"
    )

    launch_arg_yaml = load_yaml(
        config_directory / "launch_arg.yaml"
    )

    skip_state_yaml = load_yaml(
        config_directory / "skip_state.yaml"
    )

    # =========================================
    # robot.yamlからロボット設定を取得
    # =========================================

    robots = robot_yaml.get("robots", {})

    if robot_name not in robots:
        available_robots = ", ".join(
            robots.keys()
        )

        raise ValueError(
            f"robot.yamlに'{robot_name}'がありません。"
            f"使用可能なロボット: {available_robots}"
        )

    robot_config = robots[robot_name]

    # =========================================
    # skip_state.yamlからskip設定を取得
    # =========================================

    profiles = skip_state_yaml.get(
        "profiles",
        {},
    )

    if skip_profile not in profiles:
        available_profiles = ", ".join(
            profiles.keys()
        )

        raise ValueError(
            f"skip_state.yamlに'{skip_profile}'がありません。"
            f"使用可能なプロファイル: {available_profiles}"
        )

    skip_states = profiles[skip_profile]

    if skip_states is None:
        skip_states = []

    if not isinstance(skip_states, list):
        raise ValueError(
            f"skip_profile '{skip_profile}'は"
            "リスト形式で指定してください"
        )

    # =========================================
    # launch_arg.yamlから設定を取得
    # =========================================

    launches = launch_arg_yaml.get(
        "launches",
        {},
    )

    bring_me_config = launch_arg_yaml.get(
        "bring_me_node",
        {},
    )

    visual_prompt_config = launch_arg_yaml.get(
        "yoloe_visual_prompt_node",
        {},
    )
    visual_prompt_enabled = to_bool(
        visual_prompt_config.get("enabled", False)
    )
    visual_prompt_output_topic = str(
        visual_prompt_config.get("output_topic", "/yoloe/object_boxes")
    )

    actions = [
        LogInfo(
            msg=f"robot: {robot_name}"
        ),
        LogInfo(
            msg=f"skip_profile: {skip_profile}"
        ),
        LogInfo(
            msg=f"skip_states: {skip_states}"
        ),
    ]

    # =========================================
    # 別launchをIncludeして起動
    # =========================================

    # TTS
    tts_launch = create_include_launch(
        "tts",
        launches.get("tts", {}),
    )

    if tts_launch is not None:
        actions.append(tts_launch)

    # Speech Recognition
    speech_recognition_launch = create_include_launch(
        "speech_recognition",
        launches.get(
            "speech_recognition",
            {},
        ),
    )

    if speech_recognition_launch is not None:
        actions.append(
            speech_recognition_launch
        )

    # Nav2
    # 今回は追加のlaunch argumentを渡さない
    nav2_launch = create_include_launch(
        "nav2",
        launches.get("nav2", {}),
    )

    if nav2_launch is not None:
        actions.append(nav2_launch)

    # YOLO
    # カメラ画像トピックだけrobot.yamlから渡す
    image_topic_name = require_value(
        robot_config,
        "image_topic_name",
        f"robot.yamlの{robot_name}",
    )

    # Visual Promptノードがpersonも同時検出するため、同時有効時は通常YOLOを
    # 二重起動しない。
    if not visual_prompt_enabled:
        yolo_launch = create_include_launch(
            "yolo",
            launches.get("yolo", {}),
            additional_arguments={
                "image_topic_name": image_topic_name,
            },
        )

        if yolo_launch is not None:
            actions.append(yolo_launch)

    # YOLOE Visual Prompt (reference image + bbox)
    if visual_prompt_enabled:
        visual_prompt_params_file = config_directory / str(
            visual_prompt_config.get(
                "params_file", "yoloe_visual_prompt.yaml"
            )
        )
        if not visual_prompt_params_file.exists():
            raise FileNotFoundError(
                "YOLOE Visual Prompt parameter file was not found: "
                f"{visual_prompt_params_file}"
            )

        actions.append(
            Node(
                package=PACKAGE_NAME,
                executable="yoloe_visual_prompt_node",
                name="yoloe_visual_prompt_node",
                output="screen",
                parameters=[
                    str(visual_prompt_params_file),
                    {
                        "image_topic": image_topic_name,
                        "output_topic": visual_prompt_output_topic,
                    },
                ],
            )
        )

    # HSRB Library
    hsrb_library_launch = create_include_launch(
        "hsrb_library",
        launches.get(
            "hsrb_library",
            {},
        ),
    )

    if hsrb_library_launch is not None:
        actions.append(
            hsrb_library_launch
        )

    # =========================================
    # bring_me_nodeをNode()で直接起動
    # =========================================

    bring_me_enabled = to_bool(
        bring_me_config.get(
            "enabled",
            True,
        )
    )

    if not bring_me_enabled:
        return actions

    enable_nav = to_bool(
        bring_me_config.get(
            "enable_nav",
            True,
        )
    )

    # bring_me_nodeへコマンドライン引数として渡すもの
    node_arguments = [
        "--target-object",
        str(
            bring_me_config.get(
                "target_object",
                "sponge",
            )
        ),
        "--location-file",
        str(
            bring_me_config.get(
                "location_file",
                "locations.yaml",
            )
        ),
        "--task-params-file",
        str(
            bring_me_config.get(
                "task_params_file",
                "task_params.yaml",
            )
        ),
    ]

    if enable_nav:
        node_arguments.append(
            "--enable-nav"
        )

    if skip_states:
        node_arguments.append(
            "--skip-states"
        )

        node_arguments.extend(
            str(state)
            for state in skip_states
        )

    # bring_me_nodeへROSパラメータとして渡すもの
    node_parameters = {
        "enable_nav": enable_nav,

        "cmd_vel_topic": require_value(
            robot_config,
            "cmd_vel_topic",
            f"robot.yamlの{robot_name}",
        ),

        "detection_topic": (
            visual_prompt_output_topic
            if visual_prompt_enabled
            else require_value(
                robot_config,
                "detection_topic",
                f"robot.yamlの{robot_name}",
            )
        ),

        "human_detection_topic": (
            visual_prompt_output_topic
            if visual_prompt_enabled
            else require_value(
                robot_config,
                "human_detection_topic",
                f"robot.yamlの{robot_name}",
            )
        ),

        "depth_topic": require_value(
            robot_config,
            "depth_topic",
            f"robot.yamlの{robot_name}",
        ),

        "camera_info_topic": require_value(
            robot_config,
            "camera_info_topic",
            f"robot.yamlの{robot_name}",
        ),

        "scan_topic": robot_config.get(
            "scan_topic",
            "/scan",
        ),
    }

    bring_me_node = Node(
        package=PACKAGE_NAME,
        executable="bring_me_node",
        name="bring_me_node",
        output="screen",
        parameters=[
            node_parameters
        ],
        arguments=node_arguments,
    )

    actions.append(bring_me_node)

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="hsrb",
                description=(
                    "robot.yamlに登録されているロボット名"
                ),
            ),

            DeclareLaunchArgument(
                "skip_profile",
                default_value="none",
                description=(
                    "skip_state.yamlに登録されている"
                    "skipプロファイル名"
                ),
            ),

            OpaqueFunction(
                function=launch_setup
            ),
        ]
    )
