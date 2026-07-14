# Visual Prompt参照画像

このディレクトリには、YOLOE Visual Promptで検出したい物体の参照画像を保存します。例えばスポンジを検出する場合は、`sponge.jpg`を配置します。

参照画像は`colcon build`によってパッケージのshareディレクトリへインストールされます。`config/yoloe_visual_prompt.yaml`では、次のようにパッケージ内の相対パスを指定できます。

```yaml
prompt_image_path: visual_prompts/sponge.jpg
```

## 対象物のbboxを作成する

ワークスペースをビルドして`install/setup.bash`を読み込んだ後、次のコマンドを実行します。

```bash
cd ~/sobits_open_ws
source install/setup.bash
ros2 run sobits_open_mine visual_prompt_selector \
  --image src/sobits_open_mine/visual_prompts/sponge.jpg
```

表示された画像上で、検出対象だけを囲むようにマウスで矩形を選択します。選択結果は画像と同じ場所に`sponge.bbox.yaml`として保存されます。

```text
visual_prompts/
├── sponge.jpg
└── sponge.bbox.yaml
```

bboxは元画像のピクセル座標を使った`[x1, y1, x2, y2]`形式です。

`config/yoloe_visual_prompt.yaml`の`prompt_bbox`をすべて`0.0`にすると、画像と同名の`.bbox.yaml`が自動的に読み込まれます。

```yaml
prompt_image_path: visual_prompts/sponge.jpg
prompt_bbox: [0.0, 0.0, 0.0, 0.0]
prompt_bbox_file: ""
target_class_name: object
text_prompt_classes: [person]
```

- `target_class_name`: 参照画像から検出する物体の出力クラス名
- `text_prompt_classes`: Visual Promptと同時にテキスト指定で検出するクラス
- `conf`: 検出信頼度の閾値
- `iou`: NMSで使用するIoU閾値
- `device`: `cpu`、`cuda`、`0`などの推論デバイス

画像やbboxを追加・変更した後は再ビルドします。

```bash
cd ~/sobits_open_ws
colcon build --packages-select sobits_open_mine --symlink-install
source install/setup.bash
```

Visual Promptをシステム起動時に使うには、`config/launch_arg.yaml`で次を設定します。

```yaml
yoloe_visual_prompt_node:
  enabled: true
```

有効時は検出結果が`/yoloe/object_boxes`へ`vision_msgs/msg/Detection2DArray`として送信され、`bring_me_node`の物体検出と人検出に使用されます。
