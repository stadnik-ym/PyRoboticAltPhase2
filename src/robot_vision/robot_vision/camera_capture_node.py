#!/usr/bin/env python3

import json
import time

import cv2
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__("camera_capture_node")

        # =========================================================
        # PARAMETERS
        # =========================================================

        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)

        fps_descriptor = ParameterDescriptor(
            description="Camera FPS. Accepts int or float.",
            dynamic_typing=True,
        )
        self.declare_parameter("fps", 15.0, fps_descriptor)

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("status_topic", "/camera/status")
        self.declare_parameter("frame_id", "camera_frame")

        # auto / libcamera / v4l2 / custom
        self.declare_parameter("gst_source", "libcamera")

        # Якщо хочеш вручну задати pipeline з launch
        self.declare_parameter("gstreamer_pipeline", "")

        self.declare_parameter("publish_status", True)
        self.declare_parameter("retry_period_sec", 5.0)

        self.declare_parameter("warmup_read_attempts", 25)
        self.declare_parameter("warmup_sleep_sec", 0.05)

        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("flip_vertical", False)

        # =========================================================
        # READ PARAMETERS
        # =========================================================

        self.device = str(self.get_parameter("device").value)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)

        self.fps = float(self.get_parameter("fps").value)
        if self.fps <= 0.0:
            self.fps = 15.0

        self.fps_int = max(1, int(round(self.fps)))

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.gst_source = str(self.get_parameter("gst_source").value).lower().strip()
        self.custom_pipeline = str(self.get_parameter("gstreamer_pipeline").value)

        self.publish_status_enabled = bool(
            self.get_parameter("publish_status").value
        )

        self.retry_period_sec = float(
            self.get_parameter("retry_period_sec").value
        )

        self.warmup_read_attempts = int(
            self.get_parameter("warmup_read_attempts").value
        )

        self.warmup_sleep_sec = float(
            self.get_parameter("warmup_sleep_sec").value
        )

        self.flip_horizontal = bool(
            self.get_parameter("flip_horizontal").value
        )

        self.flip_vertical = bool(
            self.get_parameter("flip_vertical").value
        )

        # =========================================================
        # STATE
        # =========================================================

        self.bridge = CvBridge()
        self.cap = None

        self.camera_ok = False
        self.active_pipeline_name = ""
        self.active_pipeline_text = ""

        self.last_error = "not_started"

        self.frame_counter = 0
        self.failed_reads = 0

        self.last_retry_time = 0.0
        self.last_status_time = 0.0

        # =========================================================
        # ROS
        # =========================================================

        self.image_pub = self.create_publisher(
            Image,
            self.image_topic,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        timer_period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"✅ camera_capture_node started | "
            f"backend=GSTREAMER, "
            f"gst_source={self.gst_source}, "
            f"device={self.device}, "
            f"size={self.width}x{self.height}, "
            f"fps={self.fps}, "
            f"topic={self.image_topic}"
        )

        self.start_pipeline()

    # =========================================================
    # PIPELINES
    # =========================================================

    def build_pipelines(self):
        pipelines = []

        if self.custom_pipeline.strip():
            pipelines.append(("custom", self.custom_pipeline.strip()))

        # =========================================================
        # Raspberry Pi Camera / IMX708
        #
        # ВАЖЛИВО:
        # Без format=RGBx/BGRx libcamerasrc може вибрати RAW Bayer:
        # SBGGR/RAW, який OpenCV через appsink нормально не читає.
        # Тому примусово просимо готовий video/x-raw RGBx/BGRx.
        # =========================================================

        libcamera_rgbx_pipeline = (
            "libcamerasrc ! "
            f"video/x-raw,format=RGBx,width={self.width},height={self.height},framerate={self.fps_int}/1 ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        libcamera_bgrx_pipeline = (
            "libcamerasrc ! "
            f"video/x-raw,format=BGRx,width={self.width},height={self.height},framerate={self.fps_int}/1 ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        # Іноді на Raspberry Pi 5 стабільніше заводиться через 1536x864,
        # а вже потім resize в OpenCV/vision. Лишаємо як fallback.
        libcamera_rgbx_1536_pipeline = (
            "libcamerasrc ! "
            f"video/x-raw,format=RGBx,width=1536,height=864,framerate={self.fps_int}/1 ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "videoscale ! "
            f"video/x-raw,width={self.width},height={self.height} ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        # Запасний v4l2 pipeline. Для твоєї Pi Camera він, скоріш за все,
        # не буде основним, але нехай буде як fallback.
        v4l2_raw_pipeline = (
            f"v4l2src device={self.device} ! "
            f"video/x-raw,width={self.width},height={self.height},framerate={self.fps_int}/1 ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        v4l2_mjpg_pipeline = (
            f"v4l2src device={self.device} ! "
            f"image/jpeg,width={self.width},height={self.height},framerate={self.fps_int}/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        if self.gst_source == "custom":
            return pipelines

        if self.gst_source == "libcamera":
            pipelines.append(("libcamera_rgbx", libcamera_rgbx_pipeline))
            pipelines.append(("libcamera_bgrx", libcamera_bgrx_pipeline))
            pipelines.append(("libcamera_rgbx_1536_scaled", libcamera_rgbx_1536_pipeline))

        elif self.gst_source == "v4l2":
            pipelines.append(("v4l2_raw", v4l2_raw_pipeline))
            pipelines.append(("v4l2_mjpg", v4l2_mjpg_pipeline))

        else:
            # auto
            pipelines.append(("libcamera_rgbx", libcamera_rgbx_pipeline))
            pipelines.append(("libcamera_bgrx", libcamera_bgrx_pipeline))
            pipelines.append(("libcamera_rgbx_1536_scaled", libcamera_rgbx_1536_pipeline))
            pipelines.append(("v4l2_raw", v4l2_raw_pipeline))
            pipelines.append(("v4l2_mjpg", v4l2_mjpg_pipeline))

        return pipelines

    # =========================================================
    # START / STOP
    # =========================================================

    def start_pipeline(self):
        self.close_camera()

        pipelines = self.build_pipelines()

        if len(pipelines) == 0:
            self.camera_ok = False
            self.last_error = "no_pipeline_candidates"
            self.get_logger().error("❌ No GStreamer pipeline candidates")
            return False

        for name, pipeline in pipelines:
            self.get_logger().info(f"📷 Trying GStreamer pipeline: {name}")
            self.get_logger().info(pipeline)

            try:
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            except Exception as e:
                self.get_logger().warn(f"⚠️ VideoCapture exception for {name}: {e}")
                continue

            if not cap.isOpened():
                self.get_logger().warn(f"⚠️ Pipeline did not open: {name}")
                try:
                    cap.release()
                except Exception:
                    pass
                continue

            frame = self.try_read_warmup(cap)

            if frame is None:
                self.get_logger().warn(f"⚠️ Pipeline opened but no frame: {name}")
                try:
                    cap.release()
                except Exception:
                    pass
                continue

            self.cap = cap
            self.camera_ok = True

            self.active_pipeline_name = name
            self.active_pipeline_text = pipeline

            self.last_error = "ok"
            self.failed_reads = 0

            self.get_logger().info(
                f"✅ Camera pipeline started successfully: {name}"
            )

            return True

        self.camera_ok = False
        self.active_pipeline_name = ""
        self.active_pipeline_text = ""
        self.last_error = "failed_to_start_gstreamer_pipeline"

        self.get_logger().error(
            "❌ Failed to start any GStreamer pipeline. Will retry."
        )

        return False

    def try_read_warmup(self, cap):
        for _ in range(max(1, self.warmup_read_attempts)):
            ok, frame = cap.read()

            if ok and frame is not None and frame.size > 0:
                return frame

            time.sleep(self.warmup_sleep_sec)

        return None

    def close_camera(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

        self.cap = None
        self.camera_ok = False

    # =========================================================
    # TIMER
    # =========================================================

    def timer_callback(self):
        now = time.time()

        if self.cap is None or not self.cap.isOpened():
            self.camera_ok = False

            if now - self.last_retry_time >= self.retry_period_sec:
                self.last_retry_time = now
                self.get_logger().warn("🔁 Camera pipeline not opened. Retrying...")
                self.start_pipeline()

            self.publish_status(False, "pipeline_not_opened")
            return

        ok, frame = self.cap.read()

        if not ok or frame is None or frame.size == 0:
            self.failed_reads += 1
            self.camera_ok = False
            self.last_error = "frame_read_failed"

            if self.failed_reads == 1 or self.failed_reads % 30 == 0:
                self.get_logger().warn(
                    f"⚠️ Camera frame read failed | "
                    f"count={self.failed_reads}, "
                    f"pipeline={self.active_pipeline_name}"
                )

            if self.failed_reads >= 60:
                self.get_logger().warn("🔁 Too many failed reads. Restarting pipeline...")
                self.start_pipeline()

            self.publish_status(False, "frame_read_failed")
            return

        self.failed_reads = 0
        self.camera_ok = True
        self.last_error = "ok"

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        if self.flip_vertical:
            frame = cv2.flip(frame, 0)

        self.publish_image(frame)
        self.publish_status(True, "ok")

    # =========================================================
    # PUBLISH
    # =========================================================

    def publish_image(self, frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            self.image_pub.publish(msg)
            self.frame_counter += 1

        except Exception as e:
            self.last_error = f"publish_image_failed: {e}"
            self.get_logger().warn(f"⚠️ Failed to publish image: {e}")

    def publish_status(self, ok, reason):
        if not self.publish_status_enabled:
            return

        now = time.time()

        # Не спамимо статусом
        if now - self.last_status_time < 1.0:
            return

        self.last_status_time = now

        status = {
            "ok": bool(ok),
            "reason": str(reason),
            "device": self.device,
            "gst_source": self.gst_source,
            "active_pipeline": self.active_pipeline_name,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "image_topic": self.image_topic,
            "frame_counter": self.frame_counter,
            "failed_reads": self.failed_reads,
            "last_error": self.last_error,
            "stamp": now,
        }

        self.status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

    # =========================================================
    # DESTROY
    # =========================================================

    def destroy_node(self):
        self.close_camera()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CameraCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_camera()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()