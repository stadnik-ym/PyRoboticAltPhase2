#!/usr/bin/env python3

import json
import math
import time

import cv2
import numpy as np

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32, Float32


def has_cv2_aruco():
    """
    Перевірка, чи реально доступний cv2.aruco.
    На Raspberry часто є cv2, але немає aruco-модуля.
    """
    if not hasattr(cv2, "aruco"):
        return False

    required_attrs = [
        "getPredefinedDictionary",
        "detectMarkers",
    ]

    for attr in required_attrs:
        if not hasattr(cv2.aruco, attr):
            return False

    return True


HAS_ARUCO = has_cv2_aruco()


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        # ---------------- PARAMETERS ----------------

        self.declare_parameter("input_image_topic", "/image_raw")
        self.declare_parameter("processed_image_topic", "/aruco/debug_image")

        self.declare_parameter("status_topic", "/aruco/status")
        self.declare_parameter("found_topic", "/aruco/found")
        self.declare_parameter("id_topic", "/aruco/id")
        self.declare_parameter("offset_topic", "/aruco/offset")
        self.declare_parameter("width_topic", "/aruco/width")

        fps_descriptor = ParameterDescriptor(
            description="Camera FPS. Accepts int or float.",
            dynamic_typing=True,
        )
        self.declare_parameter("fps", 20.0, fps_descriptor)

        # ВАЖЛИВО:
        # Булева змінна НЕ називається self.publish_not_found,
        # бо нижче є метод publish_not_found(...).
        self.declare_parameter("publish_not_found", True)

        self.declare_parameter("publish_debug_image", True)

        # Якщо cv2.aruco є — можна використовувати справжній ArUco.
        # Якщо його немає — автоматично буде CONTOUR fallback.
        self.declare_parameter("use_cv2_aruco", True)

        # Словник для cv2.aruco, якщо модуль доступний.
        self.declare_parameter("aruco_dictionary", "DICT_ARUCO_ORIGINAL")

        # У CONTOUR mode ID реально зчитати неможливо,
        # тому публікуємо цей fallback ID.
        self.declare_parameter("contour_marker_id", 0)

        # Фільтри контурів
        self.declare_parameter("min_contour_area", 1200.0)
        self.declare_parameter("max_contour_area", 200000.0)
        self.declare_parameter("min_square_ratio", 0.55)
        self.declare_parameter("max_square_ratio", 1.45)

        # Якщо картинка шумна — можна піднімати blur_kernel до 5 або 7.
        # Має бути непарне число.
        self.declare_parameter("blur_kernel", 5)

        # Відмальовка рамки в повну висоту.
        # Ширина лишається по маркеру, висота — на весь кадр.
        self.declare_parameter("draw_full_height_frame", True)

        # Якщо треба зробити рамку ширшою навколо маркера.
        self.declare_parameter("frame_width_scale", 1.0)

        # ---------------- READ PARAMETERS ----------------

        self.input_image_topic = str(
            self.get_parameter("input_image_topic").value
        )
        self.processed_image_topic = str(
            self.get_parameter("processed_image_topic").value
        )

        self.status_topic = str(self.get_parameter("status_topic").value)
        self.found_topic = str(self.get_parameter("found_topic").value)
        self.id_topic = str(self.get_parameter("id_topic").value)
        self.offset_topic = str(self.get_parameter("offset_topic").value)
        self.width_topic = str(self.get_parameter("width_topic").value)

        self.fps = float(self.get_parameter("fps").value)

        if self.fps <= 0:
            self.get_logger().warn("⚠️ Invalid fps value. Fallback to 20.0")
            self.fps = 20.0

        self.publish_not_found_enabled = bool(
            self.get_parameter("publish_not_found").value
        )

        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.use_cv2_aruco = bool(
            self.get_parameter("use_cv2_aruco").value
        )

        self.aruco_dictionary_name = str(
            self.get_parameter("aruco_dictionary").value
        )

        self.contour_marker_id = int(
            self.get_parameter("contour_marker_id").value
        )

        self.min_contour_area = float(
            self.get_parameter("min_contour_area").value
        )
        self.max_contour_area = float(
            self.get_parameter("max_contour_area").value
        )
        self.min_square_ratio = float(
            self.get_parameter("min_square_ratio").value
        )
        self.max_square_ratio = float(
            self.get_parameter("max_square_ratio").value
        )

        self.blur_kernel = int(self.get_parameter("blur_kernel").value)
        if self.blur_kernel < 1:
            self.blur_kernel = 1
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1

        self.draw_full_height_frame = bool(
            self.get_parameter("draw_full_height_frame").value
        )

        self.frame_width_scale = float(
            self.get_parameter("frame_width_scale").value
        )
        if self.frame_width_scale < 0.2:
            self.frame_width_scale = 0.2

        # ---------------- STATE ----------------

        self.bridge = CvBridge()

        self.last_frame = None
        self.last_image_stamp = None
        self.last_image_rx_time = None

        self.aruco_dictionary = None
        self.aruco_params = None
        self.aruco_detector = None

        self.mode = "CONTOUR"

        if HAS_ARUCO and self.use_cv2_aruco:
            ok = self.init_aruco()
            if ok:
                self.mode = "ARUCO"
            else:
                self.mode = "CONTOUR"
        else:
            self.mode = "CONTOUR"

        # ---------------- ROS INTERFACES ----------------

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.found_pub = self.create_publisher(
            Bool,
            self.found_topic,
            10,
        )

        self.id_pub = self.create_publisher(
            Int32,
            self.id_topic,
            10,
        )

        self.offset_pub = self.create_publisher(
            Float32,
            self.offset_topic,
            10,
        )

        self.width_pub = self.create_publisher(
            Float32,
            self.width_topic,
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            self.processed_image_topic,
            10,
        )

        timer_period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(
            timer_period,
            self.detect_timer_callback,
        )

        aruco_state = "YES cv2.aruco" if HAS_ARUCO else "NO cv2.aruco"

        self.get_logger().info(
            f"✅ aruco_detector_node started in {self.mode} mode | "
            f"{aruco_state} | "
            f"input={self.input_image_topic}, "
            f"status={self.status_topic}, "
            f"fps={self.fps}"
        )

    # ------------------------------------------------------------
    # INIT ARUCO
    # ------------------------------------------------------------

    def init_aruco(self):
        try:
            dictionary_id = getattr(
                cv2.aruco,
                self.aruco_dictionary_name,
                None,
            )

            if dictionary_id is None:
                self.get_logger().warn(
                    f"⚠️ Unknown aruco_dictionary={self.aruco_dictionary_name}. "
                    f"Fallback to DICT_ARUCO_ORIGINAL."
                )
                dictionary_id = cv2.aruco.DICT_ARUCO_ORIGINAL

            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(
                dictionary_id
            )

            if hasattr(cv2.aruco, "DetectorParameters"):
                self.aruco_params = cv2.aruco.DetectorParameters()
            elif hasattr(cv2.aruco, "DetectorParameters_create"):
                self.aruco_params = cv2.aruco.DetectorParameters_create()
            else:
                self.aruco_params = None

            if hasattr(cv2.aruco, "ArucoDetector"):
                self.aruco_detector = cv2.aruco.ArucoDetector(
                    self.aruco_dictionary,
                    self.aruco_params,
                )
            else:
                self.aruco_detector = None

            return True

        except Exception as e:
            self.get_logger().warn(
                f"⚠️ cv2.aruco init failed. Fallback to CONTOUR mode. Error: {e}"
            )
            return False

    # ------------------------------------------------------------
    # IMAGE CALLBACK
    # ------------------------------------------------------------

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            self.last_frame = frame
            self.last_image_stamp = msg.header.stamp
            self.last_image_rx_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().warn(
                f"⚠️ Failed to convert image: {e}"
            )

    # ------------------------------------------------------------
    # TIMER
    # ------------------------------------------------------------

    def detect_timer_callback(self):
        if self.last_frame is None:
            self.publish_not_found("no_image")
            return

        frame = self.last_frame.copy()

        if frame is None or frame.size == 0:
            self.publish_not_found("empty_image")
            return

        try:
            if self.mode == "ARUCO":
                result = self.detect_with_aruco(frame)
            else:
                result = self.detect_with_contours(frame)

            if result is None:
                self.publish_not_found("not_found", frame)
                return

            self.publish_found(result, frame)

        except Exception as e:
            self.get_logger().error(
                f"❌ Detection error: {e}"
            )
            self.publish_not_found("detector_error", frame)

    # ------------------------------------------------------------
    # ARUCO DETECTION
    # ------------------------------------------------------------

    def detect_with_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.aruco_detector is not None:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dictionary,
                parameters=self.aruco_params,
            )

        if ids is None or len(ids) == 0 or corners is None:
            return None

        best_index = 0
        best_area = -1.0

        for i, marker_corners in enumerate(corners):
            pts = marker_corners.reshape((4, 2)).astype(np.float32)
            area = abs(cv2.contourArea(pts))

            if area > best_area:
                best_area = area
                best_index = i

        pts = corners[best_index].reshape((4, 2)).astype(np.float32)
        marker_id = int(ids[best_index][0])

        x_min = float(np.min(pts[:, 0]))
        x_max = float(np.max(pts[:, 0]))
        y_min = float(np.min(pts[:, 1]))
        y_max = float(np.max(pts[:, 1]))

        h, w = frame.shape[:2]

        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0

        width_px = x_max - x_min
        height_px = y_max - y_min

        offset_px = center_x - (w / 2.0)

        return {
            "found": True,
            "id": marker_id,
            "offset_px": float(offset_px),
            "width_px": float(width_px),
            "height_px": float(height_px),
            "center_x": float(center_x),
            "center_y": float(center_y),
            "x_min": float(x_min),
            "x_max": float(x_max),
            "y_min": float(y_min),
            "y_max": float(y_max),
            "image_width": int(w),
            "image_height": int(h),
            "mode": "ARUCO",
            "corners": pts.tolist(),
        }

    # ------------------------------------------------------------
    # CONTOUR FALLBACK DETECTION
    # ------------------------------------------------------------

    def detect_with_contours(self, frame):
        h, w = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur(
            gray,
            (self.blur_kernel, self.blur_kernel),
            0,
        )

        # Для чорних маркерів на світлому фоні добре працює INV threshold.
        thresh = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            7,
        )

        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(
            thresh,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=1,
        )

        contours, _ = cv2.findContours(
            thresh,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        best = None
        best_score = -1.0

        for cnt in contours:
            area = float(cv2.contourArea(cnt))

            if area < self.min_contour_area:
                continue

            if area > self.max_contour_area:
                continue

            peri = cv2.arcLength(cnt, True)
            if peri <= 0.0:
                continue

            approx = cv2.approxPolyDP(
                cnt,
                0.04 * peri,
                True,
            )

            if len(approx) != 4:
                continue

            if not cv2.isContourConvex(approx):
                continue

            x, y, bw, bh = cv2.boundingRect(approx)

            if bw <= 0 or bh <= 0:
                continue

            ratio = float(bw) / float(bh)

            if ratio < self.min_square_ratio:
                continue

            if ratio > self.max_square_ratio:
                continue

            rect_area = float(bw * bh)
            fill_ratio = area / rect_area if rect_area > 0 else 0.0

            # Захист від випадкових тонких контурів.
            if fill_ratio < 0.25:
                continue

            # Чим більший і квадратніший контур — тим кращий кандидат.
            square_score = 1.0 - abs(1.0 - ratio)
            score = area * max(square_score, 0.1)

            if score > best_score:
                best_score = score
                best = {
                    "contour": cnt,
                    "approx": approx,
                    "x": x,
                    "y": y,
                    "w": bw,
                    "h": bh,
                    "area": area,
                    "ratio": ratio,
                    "fill_ratio": fill_ratio,
                }

        if best is None:
            return None

        x = best["x"]
        y = best["y"]
        bw = best["w"]
        bh = best["h"]

        center_x = x + bw / 2.0
        center_y = y + bh / 2.0

        offset_px = center_x - (w / 2.0)

        x_min = float(x)
        x_max = float(x + bw)
        y_min = float(y)
        y_max = float(y + bh)

        return {
            "found": True,
            "id": int(self.contour_marker_id),
            "offset_px": float(offset_px),
            "width_px": float(bw),
            "height_px": float(bh),
            "center_x": float(center_x),
            "center_y": float(center_y),
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "image_width": int(w),
            "image_height": int(h),
            "mode": "CONTOUR",
            "area": float(best["area"]),
            "ratio": float(best["ratio"]),
            "fill_ratio": float(best["fill_ratio"]),
            "approx": best["approx"].reshape(-1, 2).tolist(),
        }

    # ------------------------------------------------------------
    # PUBLISH FOUND
    # ------------------------------------------------------------

    def publish_found(self, result, frame=None):
        marker_id = int(result.get("id", -1))
        offset_px = float(result.get("offset_px", 0.0))
        width_px = float(result.get("width_px", 0.0))

        self.found_pub.publish(Bool(data=True))
        self.id_pub.publish(Int32(data=marker_id))
        self.offset_pub.publish(Float32(data=offset_px))
        self.width_pub.publish(Float32(data=width_px))

        status = {
            "found": True,
            "active": True,
            "reason": "found",
            "id": marker_id,
            "offset_px": offset_px,
            "width_px": width_px,
            "height_px": float(result.get("height_px", 0.0)),
            "center_x": float(result.get("center_x", 0.0)),
            "center_y": float(result.get("center_y", 0.0)),
            "image_width": int(result.get("image_width", 0)),
            "image_height": int(result.get("image_height", 0)),
            "mode": str(result.get("mode", self.mode)),
            "stamp": time.time(),
        }

        self.status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

        if self.publish_debug_image and frame is not None:
            debug = self.draw_debug(frame, result)
            self.publish_debug(debug)

    # ------------------------------------------------------------
    # PUBLISH NOT FOUND
    # ------------------------------------------------------------

    def publish_not_found(self, reason="not_found", frame=None):
        if not self.publish_not_found_enabled:
            return

        self.found_pub.publish(Bool(data=False))
        self.id_pub.publish(Int32(data=-1))
        self.offset_pub.publish(Float32(data=0.0))
        self.width_pub.publish(Float32(data=0.0))

        status = {
            "found": False,
            "active": False,
            "reason": str(reason),
            "id": -1,
            "offset_px": 0.0,
            "width_px": 0.0,
            "height_px": 0.0,
            "center_x": 0.0,
            "center_y": 0.0,
            "image_width": 0,
            "image_height": 0,
            "mode": self.mode,
            "stamp": time.time(),
        }

        if frame is not None:
            h, w = frame.shape[:2]
            status["image_width"] = int(w)
            status["image_height"] = int(h)

        self.status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

        if self.publish_debug_image and frame is not None:
            debug = self.draw_no_marker_debug(frame, reason)
            self.publish_debug(debug)

    # ------------------------------------------------------------
    # DEBUG DRAW
    # ------------------------------------------------------------

    def draw_debug(self, frame, result):
        debug = frame.copy()

        h, w = debug.shape[:2]

        x_min = int(result.get("x_min", 0))
        x_max = int(result.get("x_max", 0))
        y_min = int(result.get("y_min", 0))
        y_max = int(result.get("y_max", 0))

        center_x = int(result.get("center_x", 0))
        center_y = int(result.get("center_y", 0))

        marker_id = int(result.get("id", -1))
        offset_px = float(result.get("offset_px", 0.0))
        width_px = float(result.get("width_px", 0.0))
        mode = str(result.get("mode", self.mode))

        # Масштабування ширини рамки
        if self.frame_width_scale != 1.0:
            cx = (x_min + x_max) / 2.0
            current_width = x_max - x_min
            new_width = current_width * self.frame_width_scale

            x_min = int(cx - new_width / 2.0)
            x_max = int(cx + new_width / 2.0)

            x_min = max(0, x_min)
            x_max = min(w - 1, x_max)

        # Рамка на повну висоту
        if self.draw_full_height_frame:
            draw_y_min = 0
            draw_y_max = h - 1
        else:
            draw_y_min = y_min
            draw_y_max = y_max

        cv2.rectangle(
            debug,
            (x_min, draw_y_min),
            (x_max, draw_y_max),
            (0, 255, 0),
            2,
        )

        cv2.circle(
            debug,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1,
        )

        # Центральна вертикальна лінія кадру
        cv2.line(
            debug,
            (w // 2, 0),
            (w // 2, h),
            (255, 0, 0),
            1,
        )

        # Лінія до центра маркера
        cv2.line(
            debug,
            (w // 2, h // 2),
            (center_x, center_y),
            (0, 255, 255),
            2,
        )

        text_1 = f"{mode} | id={marker_id} | width={width_px:.1f}px"
        text_2 = f"offset={offset_px:.1f}px"

        cv2.putText(
            debug,
            text_1,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug,
            text_2,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return debug

    def draw_no_marker_debug(self, frame, reason):
        debug = frame.copy()

        h, w = debug.shape[:2]

        cv2.line(
            debug,
            (w // 2, 0),
            (w // 2, h),
            (255, 0, 0),
            1,
        )

        cv2.putText(
            debug,
            f"NO MARKER: {reason}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug,
            f"mode={self.mode}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        return debug

    # ------------------------------------------------------------
    # DEBUG IMAGE PUBLISH
    # ------------------------------------------------------------

    def publish_debug(self, frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8",
            )

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"

            self.debug_image_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(
                f"⚠️ Failed to publish debug image: {e}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()