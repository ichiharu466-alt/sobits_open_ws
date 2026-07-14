from __future__ import annotations

from pathlib import Path
from typing import Any

import ast
import yaml

import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from ultralytics import YOLOE
from ultralytics.models.yolo.yoloe.predict import YOLOEVPSegPredictor


class YoloeVisualPromptNode(Node):
    """YOLOE Visual Promptで物体bboxをpublishするROS2ノード。"""

    def __init__(self):
        super().__init__("yoloe_visual_prompt_node")

        # 入出力topic
        self.declare_parameter(
            "image_topic",
            "/head_rgbd_sensor/rgb/image_rect_color",
        )
        self.declare_parameter("output_topic", "/yoloe/object_boxes")
        self.declare_parameter(
            "detected_image_topic",
            "/yolo_node/detected_image",
        )

        # YOLOE設定
        self.declare_parameter("model_path", "yoloe-11s-seg.pt")
        self.declare_parameter("prompt_image_path", "")
        self.declare_parameter("prompt_bbox", [0.0, 0.0, 0.0, 0.0])  # x1, y1, x2, y2
        self.declare_parameter("prompt_bbox_file", "")
        self.declare_parameter("target_class_name", "object")
        # 複数Visual Prompt用。空の場合は上の単一対象設定を使用する。
        # ROS 2では空配列を型付きパラメータとして初期化できないため、
        # [""]を「未指定」として扱う。
        self.declare_parameter("prompt_image_paths", [""])
        self.declare_parameter("prompt_bbox_files", [""])
        self.declare_parameter("target_class_names", [""])
        self.declare_parameter("text_prompt_classes", ["person"])
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")  # "" / "cpu" / "cuda" / "0"

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.detected_image_topic = str(
            self.get_parameter("detected_image_topic").value
        )
        self.model_path = str(self.get_parameter("model_path").value)
        self.prompt_image_path = str(self.get_parameter("prompt_image_path").value)
        self.prompt_bbox = self._parse_bbox(self.get_parameter("prompt_bbox").value)
        self.prompt_bbox_file = str(self.get_parameter("prompt_bbox_file").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.prompt_image_paths = self._read_optional_string_array(
            "prompt_image_paths"
        )
        self.prompt_bbox_files = self._read_optional_string_array(
            "prompt_bbox_files"
        )
        self.target_class_names = self._read_optional_string_array(
            "target_class_names"
        )
        self.text_prompt_classes = [
            str(value)
            for value in self.get_parameter("text_prompt_classes").value
        ]
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)

        self.bridge = CvBridge()

        self.get_logger().info("========================================")
        self.get_logger().info("YOLOE Visual Prompt Node")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"output_topic: {self.output_topic}")
        self.get_logger().info(
            f"detected_image_topic: {self.detected_image_topic}"
        )
        self.get_logger().info(f"model_path: {self.model_path}")
        self.get_logger().info(f"prompt_image_path: {self.prompt_image_path}")
        self.get_logger().info(f"prompt_bbox: {self.prompt_bbox}")
        self.get_logger().info(f"prompt_bbox_file: {self.prompt_bbox_file or '(auto)'}")
        self.get_logger().info(f"target_class_name: {self.target_class_name}")
        self.get_logger().info(f"prompt_image_paths: {self.prompt_image_paths}")
        self.get_logger().info(f"prompt_bbox_files: {self.prompt_bbox_files}")
        self.get_logger().info(f"target_class_names: {self.target_class_names}")
        self.get_logger().info(f"text_prompt_classes: {self.text_prompt_classes}")
        self.get_logger().info(f"conf: {self.conf}")
        self.get_logger().info(f"iou: {self.iou}")
        self.get_logger().info(f"imgsz: {self.imgsz}")
        self.get_logger().info(f"device: {self.device}")
        self.get_logger().info("========================================")

        self.model = self._load_model()
        self._setup_visual_prompt()

        self.publisher = self.create_publisher(
            Detection2DArray,
            self.output_topic,
            10,
        )
        self.detected_image_publisher = self.create_publisher(
            Image,
            self.detected_image_topic,
            10,
        )

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("YOLOE Visual Prompt Node is ready.")


    def _parse_bbox(self, value: Any) -> list[float]:
        """ROS parameterからprompt_bboxを[x1, y1, x2, y2]として取り出す。"""
        if isinstance(value, str):
            try:
                value = ast.literal_eval(value)
            except (SyntaxError, ValueError) as e:
                raise ValueError(
                    "prompt_bbox must be a list like [x1, y1, x2, y2]"
                ) from e

        bbox = list(value)
        if len(bbox) != 4:
            raise ValueError("prompt_bbox must be [x1, y1, x2, y2]")

        return [float(v) for v in bbox]

    def _read_optional_string_array(self, parameter_name: str) -> list[str]:
        """[""]を未指定の空配列として読み替える。"""
        values = [
            str(value)
            for value in self.get_parameter(parameter_name).value
        ]
        return [] if values == [""] else values

    def _load_model(self) -> YOLOE:
        self.get_logger().info(f"Loading YOLOE model: {self.model_path}")
        return YOLOE(self.model_path)

    def _setup_visual_prompt(self) -> None:
        prompt_specs = self._get_prompt_specs()
        visual_embeddings = []

        for class_name, image_path, bbox_file, explicit_bbox in prompt_specs:
            prompt_path = self._resolve_prompt_path(image_path)
            prompt_bbox = self._resolve_prompt_bbox(
                prompt_path,
                explicit_bbox,
                bbox_file,
            )
            self.get_logger().info(
                f"Creating visual prompt embedding: {class_name} "
                f"({prompt_path}, bbox={prompt_bbox})"
            )
            visual_embeddings.append(
                self._create_visual_embedding(prompt_path, prompt_bbox)
            )

        visual_embedding = torch.cat(visual_embeddings, dim=1)
        visual_class_names = [spec[0] for spec in prompt_specs]

        class_names = list(self.text_prompt_classes) + visual_class_names
        if len(set(class_names)) != len(class_names):
            raise ValueError(f"Prompt class names must be unique: {class_names}")

        if self.text_prompt_classes:
            self.get_logger().info(
                f"Creating text prompt embeddings: {self.text_prompt_classes}"
            )
            text_embeddings = self.model.get_text_pe(self.text_prompt_classes)
            embeddings = torch.cat(
                [text_embeddings, visual_embedding.to(text_embeddings.device)],
                dim=1,
            )
        else:
            embeddings = visual_embedding

        self.model.set_classes(class_names, embeddings)
        self.class_names = class_names

        self.get_logger().info(
            f"YOLOE prompt classes are ready: {self.class_names}"
        )

    def _get_prompt_specs(
        self,
    ) -> list[tuple[str, str, str, list[float]]]:
        """複数設定を検証し、単一設定と共通の形式で返す。"""
        if not self.prompt_image_paths:
            return [
                (
                    self.target_class_name,
                    self.prompt_image_path,
                    self.prompt_bbox_file,
                    self.prompt_bbox,
                )
            ]

        count = len(self.prompt_image_paths)
        if len(self.target_class_names) != count:
            raise ValueError(
                "target_class_names must have the same length as "
                "prompt_image_paths"
            )

        if self.prompt_bbox_files and len(self.prompt_bbox_files) != count:
            raise ValueError(
                "prompt_bbox_files must be empty or have the same length as "
                "prompt_image_paths"
            )

        bbox_files = self.prompt_bbox_files or [""] * count
        invalid_names = [name for name in self.target_class_names if not name]
        if invalid_names:
            raise ValueError("target_class_names must not contain empty names")

        return [
            (class_name, image_path, bbox_file, [0.0, 0.0, 0.0, 0.0])
            for class_name, image_path, bbox_file in zip(
                self.target_class_names,
                self.prompt_image_paths,
                bbox_files,
            )
        ]

    def _create_visual_embedding(
        self,
        prompt_path: Path,
        prompt_bbox: list[float],
    ) -> torch.Tensor:
        """1枚の参照画像から1クラス分のVisual Prompt埋め込みを作る。"""
        x1, y1, x2, y2 = prompt_bbox

        if x2 <= x1 or y2 <= y1:
            raise ValueError(
                f"Invalid prompt_bbox: {prompt_bbox}. "
                "Expected [x1, y1, x2, y2]."
            )

        # YOLOE Visual Prompt用
        # bboxes: 参照画像内のbbox
        # cls: bboxに対応する仮クラスID
        # ultralytics YOLOE expects one flat bbox and one class id per prompt.
        # Extra ndarray nesting is interpreted as a batch and breaks get_vpe().
        visual_prompts: dict[str, list] = {
            "bboxes": [[x1, y1, x2, y2]],
            "cls": [0],
        }

        predict_kwargs: dict[str, Any] = {
            "source": str(prompt_path),
            "refer_image": str(prompt_path),
            "visual_prompts": visual_prompts,
            "predictor": YOLOEVPSegPredictor,
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "verbose": False,
        }

        if self.device:
            predict_kwargs["device"] = self.device

        # predict() internally creates the visual embedding and installs it in
        # the model.  With refer_image specified, it also resets the VP
        # predictor so subsequent calls use the normal segmentation predictor.
        self.model.predict(**predict_kwargs)
        return self.model.model.pe.clone()

    def _resolve_prompt_path(self, configured_path: str) -> Path:
        """Resolve absolute, current-directory, or package-share image paths."""
        prompt_path = Path(configured_path).expanduser()
        candidates = [prompt_path]

        if not prompt_path.is_absolute():
            package_share = Path(get_package_share_directory("sobits_open_mine"))
            candidates.insert(0, package_share / prompt_path)

        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()

        checked = ", ".join(str(path) for path in candidates)
        raise FileNotFoundError(f"prompt image was not found; checked: {checked}")

    def _resolve_prompt_bbox(
        self,
        prompt_path: Path,
        explicit_bbox: list[float],
        configured_bbox_file: str,
    ) -> list[float]:
        """Use an explicit valid bbox, otherwise load the selector sidecar."""
        if self._is_valid_bbox(explicit_bbox):
            return explicit_bbox

        bbox_path = (
            self._resolve_prompt_path(configured_bbox_file)
            if configured_bbox_file
            else prompt_path.with_suffix(".bbox.yaml")
        )
        if not bbox_path.is_file():
            raise FileNotFoundError(
                f"Visual prompt bbox is not set. Run: visual_prompt_selector "
                f"--image {prompt_path}"
            )

        with bbox_path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
        bbox = self._parse_bbox(data.get("prompt_bbox", []))
        if not self._is_valid_bbox(bbox):
            raise ValueError(f"Invalid prompt_bbox in {bbox_path}: {bbox}")

        self.get_logger().info(f"Loaded prompt bbox from: {bbox_path}")
        return bbox

    @staticmethod
    def _is_valid_bbox(bbox: list[float]) -> bool:
        return len(bbox) == 4 and bbox[2] > bbox[0] and bbox[3] > bbox[1]

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        predict_kwargs: dict[str, Any] = {
            "source": cv_image,
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "verbose": False,
        }

        if self.device:
            predict_kwargs["device"] = self.device

        try:
            results = self.model.predict(**predict_kwargs)
        except Exception as e:
            self.get_logger().error(f"YOLOE prediction failed: {e}")
            return

        detections_msg = self._results_to_detection_array(results, msg.header)
        self.publisher.publish(detections_msg)
        self._publish_detected_image(results, cv_image, msg.header)

    def _publish_detected_image(
        self,
        results: Any,
        cv_image: Any,
        header: Header,
    ) -> None:
        """検出結果を描画したBGR画像をPublishする。"""
        detected_image = cv_image

        try:
            if results:
                # Ultralyticsのplot()はbbox、クラス名、信頼度を描画した
                # BGR ndarrayを返す。
                detected_image = results[0].plot()

            image_msg = self.bridge.cv2_to_imgmsg(
                detected_image,
                encoding="bgr8",
            )
            image_msg.header = header
            self.detected_image_publisher.publish(image_msg)
        except Exception as e:
            # 可視化に失敗してもDetection2DArrayのPublishは継続する。
            self.get_logger().error(f"Failed to publish detected image: {e}")

    def _results_to_detection_array(
        self,
        results: Any,
        header: Header,
    ) -> Detection2DArray:
        array_msg = Detection2DArray()
        array_msg.header = header

        if results is None:
            return array_msg

        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            xyxy = getattr(boxes, "xyxy", None)
            conf = getattr(boxes, "conf", None)
            classes = getattr(boxes, "cls", None)

            if xyxy is None or conf is None or classes is None:
                continue

            xyxy_np = xyxy.detach().cpu().numpy()
            conf_np = conf.detach().cpu().numpy()
            classes_np = classes.detach().cpu().numpy()

            for box, score, class_index in zip(xyxy_np, conf_np, classes_np):
                x1, y1, x2, y2 = [float(v) for v in box]

                det = Detection2D()
                det.header = header

                bbox = BoundingBox2D()
                self._set_bbox_center(bbox, (x1 + x2) / 2.0, (y1 + y2) / 2.0)
                bbox.size_x = max(0.0, x2 - x1)
                bbox.size_y = max(0.0, y2 - y1)
                det.bbox = bbox

                hyp = ObjectHypothesisWithPose()
                class_id = int(class_index)
                hyp.hypothesis.class_id = (
                    self.class_names[class_id]
                    if 0 <= class_id < len(self.class_names)
                    else str(class_id)
                )
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)

                array_msg.detections.append(det)

        return array_msg

    def _set_bbox_center(self, bbox: BoundingBox2D, x: float, y: float) -> None:
        """vision_msgsのバージョン差を吸収してbbox中心を設定する。"""
        if hasattr(bbox.center, "position"):
            bbox.center.position.x = x
            bbox.center.position.y = y
            if hasattr(bbox.center, "theta"):
                bbox.center.theta = 0.0
        else:
            bbox.center.x = x
            bbox.center.y = y
            bbox.center.theta = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = YoloeVisualPromptNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
