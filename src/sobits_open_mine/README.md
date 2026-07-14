# sobits_open_mine

HSR（Human Support Robot）で「Bring Me」タスクを実行するROS 2パッケージです。
音声で依頼された部屋と物体を認識し、目的地への移動、物体検出、3次元位置推定、把持、操作者への受け渡し、アリーナ退出までを状態機械で順番に実行します。

## 動作の流れ

基本的な状態遷移は次のとおりです。

1. LiDARでドアが開いていることを確認する
2. 操作者との対話地点へ移動する
3. 人を検出する
4. 音声認識で部屋と対象物を取得する
5. 指定された部屋へ移動する
6. YOLOまたはYOLOE Visual Promptで対象物を検出する
7. Depth画像とCameraInfoから対象物の3次元位置を計算する
8. TFで対象位置を`base_footprint`座標系へ変換する
9. 台車とアームの位置を調整して物体を把持する
10. 操作者との対話地点へ戻る
11. 人へ物体を受け渡す
12. 出口へ移動する

## 前提

- ROS 2 Jazzy
- HSR実機または対応するシミュレーション環境
- `sobits_open_ws`がビルド済みであること
- HSRのセンサー、TF、コントローラーが利用可能であること
- 地図が作成済みで、`locations.yaml`の座標が地図と一致していること

初回またはソース変更後はワークスペースをビルドします。

```bash
cd ~/sobits_open_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 起動方法

### 1. 関連ノードの一括起動

ワークスペース直下から起動スクリプトを実行します。

```bash
cd ~/sobits_open_ws
chmod +x start_bring_me_tabs.sh  # 初回のみ
./start_bring_me_tabs.sh
```

このスクリプトはGNOME Terminalのタブを作成し、次のシステムを起動します。

| タブ | 起動するシステム | 役割 |
|---|---|---|
| TTS | `sobits_tts` | 発話 |
| Whisper | `sobits_speech_recognition` | 音声認識 |
| Nav2 | `sobits_nav` | 自律移動 |
| YOLO | `yolo_ros` | 物体・人物の2次元検出 |
| HSRB Library | `hsrb_library` | 台車・関節・手先の制御 |

各タブでは`~/sobits_open_ws/install/setup.bash`を読み込み、`hsrb_mode`を実行してからノードを起動します。

### 2. 地図上の初期位置設定

RVizの`2D Pose Estimate`を使い、表示されている地図とLiDARの点群が重なるようにロボットの初期姿勢を設定します。

位置だけでなくロボットの向きも合わせてください。地図とLiDARがずれている状態では、Nav2による移動が失敗したり、誤った経路を生成したりする可能性があります。

### 3. Bring Meタスクの開始

新しいターミナルで次を実行します。

```bash
cd ~/sobits_open_ws
source install/setup.bash
hsrb_mode
ros2 run sobits_open_mine bring_me_node --enable-nav
```

`--enable-nav`を付けない場合、直接実行時のナビゲーションは無効になるため注意してください。

対象物をあらかじめ指定する場合は、次のように実行できます。

```bash
ros2 run sobits_open_mine bring_me_node --enable-nav --target-object sponge
```

状態を限定して確認する場合は、`--skip-states`にスキップする状態名を指定できます。

```bash
ros2 run sobits_open_mine bring_me_node \
  --target-object sponge \
  --skip-states DOOR_OPEN NAVIGATE_TO_INTERACTION_POINT HUMAN_DETECTION
```

## Launchファイルによる起動

`bring_me_system.launch.py`から関連システムをまとめて起動することもできます。

```bash
ros2 launch sobits_open_mine bring_me_system.launch.py \
  robot:=hsrb \
  skip_profile:=none
```

起動対象は`config/launch_arg.yaml`で切り替えます。現在の設定では`bring_me_node.enabled`と`yoloe_visual_prompt_node.enabled`が`false`です。Launchファイルだけで一連の動作を開始する場合は、必要な項目を`true`にしてください。

## YOLOE Visual Prompt

通常のYOLOクラスでは検出しにくい物体を、参照画像と画像内の矩形領域から検出する機能です。

### 参照画像の準備

参照画像を`visual_prompts/`へ保存します。例えばスポンジの場合は次の配置にします。

```text
visual_prompts/
├── sponge.jpg
└── sponge.bbox.yaml
```

マウスで参照物体の範囲を選択します。

```bash
cd ~/sobits_open_ws
source install/setup.bash
ros2 run sobits_open_mine visual_prompt_selector \
  --image src/sobits_open_mine/visual_prompts/sponge.jpg
```

選択すると、画像と同じ場所に`sponge.bbox.yaml`が生成されます。矩形は元画像のピクセル座標を使った`[x1, y1, x2, y2]`形式です。

ファイル追加後は再ビルドします。

```bash
colcon build --packages-select sobits_open_mine --symlink-install
source install/setup.bash
```

### Visual Promptの有効化

`config/yoloe_visual_prompt.yaml`で参照画像などを設定します。

```yaml
prompt_image_path: visual_prompts/sponge.jpg
prompt_bbox: [0.0, 0.0, 0.0, 0.0]
target_class_name: object
text_prompt_classes: [person]
```

`prompt_bbox`をすべて`0.0`にすると、画像と同名の`.bbox.yaml`が自動的に読み込まれます。`person`は人検出を同時に行うためのテキストプロンプトです。

続いて`config/launch_arg.yaml`を変更します。

```yaml
yoloe_visual_prompt_node:
  enabled: true
```

Visual Promptが有効な場合は通常のYOLOを二重起動せず、`bring_me_node`の物体検出と人検出の入力が`/yoloe/object_boxes`へ切り替わります。

## 使用するAction

| Action名 | 型 | 用途 |
|---|---|---|
| `/speech_word` | `sobits_interfaces/action/TextToSpeech` | ロボットの発話 |
| `/speech_recognition` | `sobits_interfaces/action/SpeechRecognition` | 音声コマンドの認識 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 地図上の目的地への移動 |
| `/hsrb/move_to_pose` | `sobits_interfaces/action/MoveToPose` | HSRの登録済み姿勢への移動 |
| `/hsrb/move_joint` | `sobits_interfaces/action/MoveJoint` | 関節角度の指定 |
| `/hsrb/move_wheel_linear` | `sobits_interfaces/action/MoveWheelLinear` | 台車の直進移動 |
| `/hsrb/move_wheel_rotate` | `sobits_interfaces/action/MoveWheelRotate` | 台車の旋回 |

## 使用するService

| Service名 | 型 | 用途 |
|---|---|---|
| `/hsrb/get_hand_to_coord` | `sobits_interfaces/srv/GetHandToTargetCoord` | 目標座標へ手先を動かすための関節角度を取得 |
| `/hsrb/get_hand_to_tf` | `sobits_interfaces/srv/GetHandToTargetTF` | 目標TFへ手先を動かすための情報を取得 |

現在の把持処理では主に`/hsrb/get_hand_to_coord`を使用します。

## 使用するTopic

| Topic名 | 型 | 入出力 | 用途 |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscribe | ドア開閉判定と周辺距離の取得 |
| `/head_rgbd_sensor/rgb/image_rect_color` | `sensor_msgs/msg/Image` | YOLO入力 | 通常の物体・人物検出 |
| `/head_rgbd_sensor/rgb/image_raw` | `sensor_msgs/msg/Image` | Subscribe | Visual Promptの画像入力（設定で変更可能） |
| `/yolo_node/object_boxes` | `vision_msgs/msg/Detection2DArray` | Subscribe | 通常YOLOの検出結果 |
| `/yoloe/object_boxes` | `vision_msgs/msg/Detection2DArray` | Publish/Subscribe | Visual Promptの検出結果 |
| `/head_rgbd_sensor/depth_registered/image_raw` | `sensor_msgs/msg/Image` | Subscribe | 対象物までのDepth取得 |
| `/head_rgbd_sensor/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | Subscribe | 2D座標から3D座標への変換 |
| `/omni_base_controller/cmd_vel` | `geometry_msgs/msg/Twist` | Publish | 人への向き合わせと把持前の台車微調整 |
| `/gripper_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Publish | グリッパの開閉 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Subscribe | 指のeffortによる把持成功判定 |

Topic名は主に`config/robot.yaml`で変更できます。

## TF

物体検出では、Depth画像から求めたカメラ座標系の3次元点をTFで`base_footprint`へ変換します。把持時には`base_footprint`から`hand_palm_link`の位置も参照し、対象物とアームの横位置を合わせます。

そのため、カメラフレーム、`base_footprint`、`hand_palm_link`間のTFが配信されている必要があります。

## Blackboardが管理する情報

Blackboardは、状態機械の各処理間で結果を共有するための一時的なメモリです。YAMLが固定設定を保持するのに対し、Blackboardは実行中に変化する値を保持します。

| キー | 内容 |
|---|---|
| `locations` | `locations.yaml`から読み込んだ地点情報 |
| `task_params` | `task_params.yaml`から読み込んだ動作設定 |
| `skip_states` | 実行しない状態の集合 |
| `command_text` | 音声認識結果 |
| `target_room` | 音声から抽出した部屋名 |
| `target_object` | 探索・把持する物体名 |
| `destination_location` | 移動先に対応する地点名 |
| `yolo_target_labels` | 対象物として許可するYOLOラベル一覧 |
| `human_detected` | 人検出の成否 |
| `last_nav_goal` | 最後に指定したナビゲーション目標 |
| `target_detection` | 選択された2次元検出結果 |
| `target_pose_camera` | カメラ座標系での物体3次元位置 |
| `target_pose_base` | `base_footprint`座標系での物体3次元位置 |
| `grasp_failure_stage` | 把持が失敗した処理段階 |
| `grasp_failure_message` | 把持失敗の詳細 |
| `grasp_plan` | 対象座標、把持方式、成功・失敗結果のまとめ |
| `object_grasped` | 把持成功フラグ |
| `object_delivered` | 受け渡し成功フラグ |

処理終了時には主要なBlackboard値がログへ表示されます。

## YAMLファイルの役割

| ファイル | 管理内容 |
|---|---|
| `config/robot.yaml` | ロボットごとのカメラ、LiDAR、検出結果、`cmd_vel`、Action名など |
| `config/locations.yaml` | 地図座標系における対話地点、部屋、入口、出口の位置と姿勢 |
| `config/task_params.yaml` | ドア判定、音声、部屋・物体名、検出閾値、姿勢推定、把持、受け渡し、退出の動作パラメータ |
| `config/launch_arg.yaml` | 起動する外部Launch、各Launchの引数、Bring MeノードとVisual Promptノードの有効・無効 |
| `config/skip_state.yaml` | テスト目的別にスキップする状態のプロファイル |
| `config/yoloe_visual_prompt.yaml` | Visual Promptの参照画像、bbox、モデル、クラス、推論閾値、入出力Topic |

### skip profile

Launch起動時は`skip_state.yaml`に登録されたプロファイルを指定できます。

```bash
ros2 launch sobits_open_mine bring_me_system.launch.py skip_profile:=no_navigation
```

主なプロファイルは`none`、`speech_only`、`detection_test`、`grasp_test`、`no_navigation`です。

## 動作確認に使えるコマンド

```bash
ros2 topic echo /yolo_node/object_boxes
ros2 topic echo /joint_states
ros2 topic hz /head_rgbd_sensor/depth_registered/image_raw
ros2 action list
ros2 service list | grep /hsrb
ros2 run tf2_ros tf2_echo base_footprint hand_palm_link
```

実機で把持を試す場合は、非常停止できる状態を確保し、周囲とアームの可動範囲に人や障害物がないことを確認してください。

