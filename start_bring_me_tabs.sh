# 一斉にlaunchするためのshellscript gnome-terminalによってターミナルのタブ自動追加
#!/bin/bash
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

open_tab "YOLO" \
    "ros2 launch yolo_ros yolo.launch.py weights_path:=/home/su-laptop-19/sobits_open_ws/install/yolo_ros/share/yolo_ros/weights weight_file:=yoloe-26s-seg.pt use_bbox_to_3d:=false use_keypoint_to_3d:=false use_mask_to_3d:=false conf:=0.25 device:=cpu fuse:=false"

open_tab "HSRB Library" \
    "ros2 launch hsrb_library library_server.launch.py"
