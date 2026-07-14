from __future__ import annotations

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from vision_msgs.msg import Detection2DArray
import tf2_ros
import tf2_geometry_msgs
from builtin_interfaces.msg import Time
from rclpy.duration import Duration

from .ros_client import HsrbLibraryClient, NavClient, SpeechClient, TTSClient


class BringMeNode(Node):
    def __init__(self, enable_nav: bool | None = None):
        super().__init__("bring_me_node")

        # ROS communication settings. These can be overridden by --ros-args -p.
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/omni_base_controller/cmd_vel")
        self.declare_parameter(
            "detection_topic",
            "/object_yolo_node/object_boxes",
        )
        self.declare_parameter(
            "human_detection_topic",
            "/human_yolo_node/object_boxes",
        )
        self.declare_parameter("depth_topic", "/head_rgbd_sensor/depth_registered/image_raw")
        self.declare_parameter("camera_info_topic", "/head_rgbd_sensor/rgb/camera_info")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("tts_action", "/speech_word")
        self.declare_parameter("stt_action", "/speech_recognition")
        self.declare_parameter("nav_action", "/navigate_to_pose")
        self.declare_parameter("enable_nav", False)

        scan_topic = self.get_parameter("scan_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        human_detection_topic = self.get_parameter("human_detection_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.base_frame = str(self.get_parameter("base_frame").value)
        tts_action = self.get_parameter("tts_action").value
        stt_action = self.get_parameter("stt_action").value
        nav_action = self.get_parameter("nav_action").value

        if enable_nav is None:
            self.enable_nav = bool(self.get_parameter("enable_nav").value)
        else:
            self.enable_nav = bool(enable_nav)

        # Latest sensor data. task_actions read these values.
        self.latest_scan: LaserScan | None = None
        self.latest_detections: Detection2DArray | None = None
        self.latest_human_detections: Detection2DArray | None = None
        self.latest_depth_image: Image | None = None
        self.latest_camera_info: CameraInfo | None = None

        # TF buffer/listener for camera -> base pose conversion.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10,
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            detection_topic,
            self.detection_callback,
            10,
        )
        self.human_detection_sub = self.create_subscription(
            Detection2DArray,
            human_detection_topic,
            self.human_detection_callback,
            10,
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10,
        )

        # Action clients
        self.tts = TTSClient(self, tts_action)
        self.speech = SpeechClient(self, stt_action)
        self.nav = NavClient(self, nav_action)
        self.hsrb = HsrbLibraryClient(self)

        self.get_logger().info("BringMeNode initialized.")
        self.get_logger().info(f"scan_topic: {scan_topic}")
        self.get_logger().info(f"cmd_vel_topic: {cmd_vel_topic}")
        self.get_logger().info(f"detection_topic: {detection_topic}")
        self.get_logger().info(f"human_detection_topic: {human_detection_topic}")
        self.get_logger().info(f"depth_topic: {depth_topic}")
        self.get_logger().info(f"camera_info_topic: {camera_info_topic}")
        self.get_logger().info(f"base_frame: {self.base_frame}")
        self.get_logger().info(f"tts_action: {tts_action}")
        self.get_logger().info(f"stt_action: {stt_action}")
        self.get_logger().info(f"nav_action: {nav_action}")
        self.get_logger().info(f"enable_nav: {self.enable_nav}")

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def detection_callback(self, msg: Detection2DArray) -> None:
        self.latest_detections = msg

    def human_detection_callback(self, msg: Detection2DArray) -> None:
        self.latest_human_detections = msg

    def depth_callback(self, msg: Image) -> None:
        self.latest_depth_image = msg

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def say(self, text: str) -> bool:
        return self.tts.say(text)

    def listen(
        self,
        timeout_sec: int | float = 7,
        silent_mode: bool = False,
        feedback_rate: float = 0.5,
    ) -> str | None:
        return self.speech.listen(
            timeout_sec=timeout_sec,
            silent_mode=silent_mode,
            feedback_rate=feedback_rate,
        )

    def navigate_to_pose(self, pose: PoseStamped) -> bool:
        if not self.enable_nav:
            self.get_logger().warn("[NAV DISABLED] Skip navigation.")
            return True

        return self.nav.go_to(pose)

    def publish_cmd_vel(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)


    def transform_point_to_base(self, frame_id: str, x: float, y: float, z: float, timeout_sec: float = 0.5) -> dict | None:
        """Transform a 3D point from camera frame to self.base_frame.

        Returns a dict so task_actions can store it directly into the blackboard.
        """
        if not frame_id:
            self.get_logger().error("[TF] source frame_id is empty")
            return None

        point = PointStamped()
        point.header.stamp = Time()
        point.header.frame_id = str(frame_id)
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)

        try:
            transformed = self.tf_buffer.transform(point, self.base_frame, Duration(seconds=float(timeout_sec)))
        except Exception as exc:
            self.get_logger().warn(f"[TF] failed to transform {frame_id} -> {self.base_frame}: {exc}")
            return None

        return {
            "frame_id": transformed.header.frame_id,
            "x": float(transformed.point.x),
            "y": float(transformed.point.y),
            "z": float(transformed.point.z),
        }
