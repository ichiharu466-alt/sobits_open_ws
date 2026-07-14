# YOLOE Visual Prompt Node

> **現在の運用では未使用です。** Bring Meでは、人検出用の`human_yolo_node`（`yoloe-26s-seg.pt`）と物体検出用の`object_yolo_node`（`best.pt`）を使用しています。この文書は、将来YOLOE Visual Prompt方式へ切り替える場合のために残しています。

現在使用している検出構成は次のとおりです。

| 用途 | 検出結果 | 描画画像 |
|---|---|---|
| 人検出 | `/human_yolo_node/object_boxes` | `/human_yolo_node/detected_image` |
| 物体検出 | `/object_yolo_node/object_boxes` | `/object_yolo_node/detected_image` |

起動設定とBring Me側の購読先については、[ワークスペースのREADME](../../../README.md#人検出用物体検出用yolo)を参照してください。

---

[`yoloe_visual_prompt_node.py`](./yoloe_visual_prompt_node.py) は、参照画像内で指定した物体と似た物体を、YOLOE Visual Promptを使ってRGB画像から検出するROS 2ノードです。

ノード起動時に1つ以上の参照画像とbboxからVisual Prompt埋め込みを作成します。その後、カメラ画像を受信するたびに推論を行い、検出した物体のクラス名、信頼度、2D bboxを `vision_msgs/msg/Detection2DArray` としてPublishします。Visual Promptによる複数の対象物に加え、`person`などのText Promptクラスも同時に検出できます。

## 処理の流れ

1. YOLOEモデルを読み込む
2. 参照画像と、その画像内にある対象物のbboxを読み込む
3. bboxで囲まれた対象物からVisual Prompt埋め込みを作成する
4. Text Promptが指定されている場合は、その埋め込みを結合する
5. RGB画像をSubscribeして推論する
6. 検出結果を `Detection2DArray` に変換してPublishする

## 入出力

| 種別 | デフォルトのトピック | メッセージ型 | 説明 |
|---|---|---|---|
| Subscribe | `/head_rgbd_sensor/rgb/image_rect_color` | `sensor_msgs/msg/Image` | 推論対象のRGB画像 |
| Publish | `/yoloe/object_boxes` | `vision_msgs/msg/Detection2DArray` | クラス名、信頼度、2D bboxを含む検出結果 |
| Publish | `/yolo_node/detected_image` | `sensor_msgs/msg/Image` | bbox、クラス名、信頼度を描画した画像 |

出力bboxの中心とサイズは、入力画像上のピクセル座標で格納されます。入力画像の `header` は出力にも引き継がれます。

## Visual Promptの準備

検出対象が写った参照画像を `visual_prompts` ディレクトリへ配置します。次に、対象物だけを囲むbboxを選択します。

```bash
cd ~/sobits_open_ws
source install/setup.bash
ros2 run sobits_open_mine visual_prompt_selector \
  --image src/sobits_open_mine/visual_prompts/sponge.jpg
```

画像上で対象物をドラッグして囲み、`Enter`または`Space`を押すと、画像と同じ場所に `sponge.bbox.yaml` が保存されます。

```text
visual_prompts/
├── sponge.jpg
└── sponge.bbox.yaml
```

bboxは元画像のピクセル座標を使った `[x1, y1, x2, y2]` 形式です。

## パラメータ

設定ファイルは [`../config/yoloe_visual_prompt.yaml`](../config/yoloe_visual_prompt.yaml) です。

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `image_topic` | `/head_rgbd_sensor/rgb/image_rect_color` | 入力RGB画像のトピック |
| `output_topic` | `/yoloe/object_boxes` | 検出結果の出力トピック |
| `detected_image_topic` | `/yolo_node/detected_image` | 描画済み検出画像の出力トピック |
| `model_path` | `yoloe-11s-seg.pt` | YOLOEモデルのパス |
| `prompt_image_path` | 空文字列 | Visual Promptに使う参照画像のパス |
| `prompt_bbox` | `[0.0, 0.0, 0.0, 0.0]` | 参照画像内の対象物bbox |
| `prompt_bbox_file` | 空文字列 | bbox YAMLのパス。空の場合は参照画像と同名の `.bbox.yaml` を使う |
| `target_class_name` | `object` | Visual Promptで検出した物体の出力クラス名 |
| `prompt_image_paths` | `[]` | 複数対象の参照画像パス一覧。指定すると単一対象設定より優先される |
| `prompt_bbox_files` | `[""]` | 複数対象のbbox YAMLパス一覧。空文字列なら各画像と同名のファイルを使う |
| `target_class_names` | `[]` | 複数対象の出力クラス名一覧 |
| `text_prompt_classes` | `[person]` | 同時に検出するText Promptクラスの一覧 |
| `conf` | `0.25` | 検出信頼度の閾値 |
| `iou` | `0.7` | NMSで使用するIoU閾値 |
| `imgsz` | `640` | 推論時の入力画像サイズ |
| `device` | 空文字列 | 推論デバイス。`cpu`、`cuda`、`0`などを指定 |

`prompt_image_path` には絶対パスのほか、パッケージshareディレクトリからの相対パスを指定できます。

```yaml
yoloe_visual_prompt_node:
  ros__parameters:
    prompt_image_path: visual_prompts/sponge.jpg
    prompt_bbox: [0.0, 0.0, 0.0, 0.0]
    prompt_bbox_file: ""
    target_class_name: sponge
    text_prompt_classes: [person]
```

`prompt_bbox`をすべて `0.0` にすると、参照画像と同名のbbox YAMLが自動的に読み込まれます。

複数のVisual Prompt対象を同時検出する場合は、画像とクラス名を同じ順番で指定します。

```yaml
prompt_image_paths:
  - visual_prompts/air_freshener.jpg
  - visual_prompts/chips.jpg
  - visual_prompts/sponge.jpg
  - visual_prompts/tea.jpg
prompt_bbox_files: ["", "", "", ""]
target_class_names: [air_freshener, chips, sponge, tea]
text_prompt_classes: [person]
```

`prompt_image_paths`、`target_class_names`、および指定する場合の`prompt_bbox_files`は、要素数と並びを一致させてください。`prompt_image_paths`が空の場合は、従来の`prompt_image_path`、`prompt_bbox`、`prompt_bbox_file`、`target_class_name`が使われます。

## 起動方法

```bash
cd ~/sobits_open_ws
colcon build --packages-select sobits_open_mine --symlink-install
source install/setup.bash
ros2 run sobits_open_mine yoloe_visual_prompt_node \
  --ros-args \
  --params-file src/sobits_open_mine/config/yoloe_visual_prompt.yaml
```

Visual Prompt方式へ切り替えてBring Meシステムと一緒に起動する場合は、[`../config/launch_arg.yaml`](../config/launch_arg.yaml) でノードを有効にします。現在の設定は`false`です。

```yaml
yoloe_visual_prompt_node:
  enabled: true
```

## 検出結果の確認

```bash
ros2 topic echo /yoloe/object_boxes
```

描画済み画像は`rqt_image_view`などで確認できます。

```bash
ros2 run rqt_image_view rqt_image_view /yolo_node/detected_image
```

検出結果がないフレームでも、空の `Detection2DArray` と描画なしの入力画像がPublishされます。

## 注意事項

- `prompt_image_path`には実在する画像を指定してください。
- bboxは対象物を正確に囲み、`x2 > x1`かつ`y2 > y1`となるようにしてください。
- `target_class_name`と`text_prompt_classes`のクラス名は重複できません。
- 参照画像やbboxを追加・変更した後は、パッケージを再ビルドしてください。
- モデルの初回取得が必要な環境では、起動時にネットワーク接続が必要になる場合があります。
- 時間が足りず本当に理解できてない部分も多いのでちょっと今後精進していきます
