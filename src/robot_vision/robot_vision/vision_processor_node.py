#!/usr/bin/env python3

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__("vision_processor_node")

        self.declare_parameter("input_image_topic", "/image_raw")
        self.declare_parameter("aruco_status_topic", "/aruco/status")
        self.declare_parameter("processed_image_topic", "/image_processed")

        self.declare_parameter("center_deadzone_px", 60.0)
        self.declare_parameter("status_timeout_sec", 0.5)
        self.declare_parameter("publish_every_n", 1)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.aruco_status_topic = str(self.get_parameter("aruco_status_topic").value)
        self.processed_image_topic = str(self.get_parameter("processed_image_topic").value)

        self.center_deadzone_px = float(self.get_parameter("center_deadzone_px").value)
        self.status_timeout_sec = float(self.get_parameter("status_timeout_sec").value)
        self.publish_every_n = max(1, int(self.get_parameter("publish_every_n").value))

        self.bridge = CvBridge()

        self.last_status = None
        self.last_status_rx_t = 0.0
        self.frame_counter = 0

        self.image_pub = self.create_publisher(
            Image,
            self.processed_image_topic,
            qos_profile_sensor_data,
        )

        self.status_sub = self.create_subscription(
            String,
            self.aruco_status_topic,
            self.status_callback,
            10,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"✅ vision_processor_node started | {self.input_image_topic} + "
            f"{self.aruco_status_topic} -> {self.processed_image_topic}"
        )

    def status_callback(self, msg: String):
        try:
            self.last_status = json.loads(msg.data)
            self.last_status_rx_t = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Bad aruco status JSON: {e}")

    def image_callback(self, msg: Image):
        self.frame_counter += 1
        if self.frame_counter % self.publish_every_n != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"⚠️ cv_bridge error: {e}")
            return

        h, w = frame.shape[:2]
        cx = w // 2

        # Center line
        cv2.line(frame, (cx, 0), (cx, h), (255, 255, 255), 1)

        dz = int(self.center_deadzone_px)
        cv2.line(frame, (cx - dz, 0), (cx - dz, h), (120, 120, 120), 1)
        cv2.line(frame, (cx + dz, 0), (cx + dz, h), (120, 120, 120), 1)

        status_fresh = (
            self.last_status is not None
            and (time.monotonic() - self.last_status_rx_t) <= self.status_timeout_sec
        )

        if status_fresh and self.last_status.get("found", False):
            corners = self.last_status.get("corners", [])

            if len(corners) == 4:
                pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            marker_cx = int(self.last_status.get("cx", 0))
            marker_cy = int(self.last_status.get("cy", 0))
            cv2.circle(frame, (marker_cx, marker_cy), 5, (0, 255, 0), -1)

            txt1 = (
                f"ARUCO FOUND | id={self.last_status.get('id', -1)} | "
                f"offset={self.last_status.get('offset_x', 0.0):.1f}px | "
                f"width={self.last_status.get('width', 0.0):.1f}px"
            )
            cv2.putText(
                frame,
                txt1,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame,
                "ARUCO NOT FOUND",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            frame,
            "vision_processor: debug image only | control = mission_master",
            (10, h - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self.image_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()