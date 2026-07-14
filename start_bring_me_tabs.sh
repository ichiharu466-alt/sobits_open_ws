#!/bin/bash
# Bring Meに必要なノードをgnome-terminalのタブで一斉起動する。
# YOLOは人検出用（26s）と物体検出用（best.pt）を別ノードで起動する。

open_tab() {
    local title="$1"
    local command="$2"

    gnome-terminal \
        --tab \
        --title="$title" \
        -- bash -ic "
            echo '[$title] HSRBモードへ切り替えます'
            source ~/sobits_open_ws/install/setup.bash
            hsrb_mode

            echo
            echo '実行コマンド:'
            echo '$command'
            echo

            $command

            echo
            echo '[$title] プロセスが終了しました'
            exec bash -i
        "
}

open_tab "TTS" \
    "ros2 launch sobits_tts kokoro.launch.py kokoro_lang_code:=j kokoro_voice:=jf_alpha"

open_tab "Whisper" \
    "ros2 launch sobits_speech_recognition whisper.launch.py"

open_tab "Nav2" \
    "ros2 launch sobits_nav nav2.launch.py"

# 人検出用YOLO
open_tab "YOLO Human" \
    "ros2 launch yolo_ros yolo.launch.py node_name:=human_yolo_node image_topic_name:=/head_rgbd_sensor/rgb/image_rect_color weights_path:=/home/su-laptop-19/sobits_open_ws/src/yolo_ros/weights weight_file:=yoloe-26s-seg.pt use_bbox_to_3d:=false use_keypoint_to_3d:=false use_mask_to_3d:=false conf:=0.25 device:=cpu fuse:=false"

# 物体検出用YOLO
open_tab "YOLO Object" \
    "ros2 launch yolo_ros yolo.launch.py node_name:=object_yolo_node image_topic_name:=/head_rgbd_sensor/rgb/image_rect_color weights_path:=/home/su-laptop-19/sobits_open_ws/src/yolo_ros/weights weight_file:=best.pt use_bbox_to_3d:=false use_keypoint_to_3d:=false use_mask_to_3d:=false conf:=0.25 device:=cpu fuse:=false"

# YOLOE Visual Promptは現在使用しない。
# open_tab "YOLOE Visual Prompt" \
#     "ros2 run sobits_open_mine yoloe_visual_prompt_node --ros-args --params-file /home/su-laptop-19/sobits_open_ws/src/sobits_open_mine/config/yoloe_visual_prompt.yaml"

open_tab "HSRB Library" \
    "ros2 launch hsrb_library library_server.launch.py"
