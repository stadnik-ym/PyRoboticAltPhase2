#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__("lidar_avoidance_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("request_topic", "/avoidance/request")
        self.declare_parameter("cmd_topic", "/avoidance/cmd_vel")
        self.declare_parameter("status_topic", "/avoidance/status")

        self.declare_parameter("front_min_deg", -35.0)
        self.declare_parameter("front_max_deg", 35.0)

        self.declare_parameter("left_min_deg", 35.0)
        self.declare_parameter("left_max_deg", 115.0)

        self.declare_parameter("right_min_deg", -115.0)
        self.declare_parameter("right_max_deg", -35.0)

        self.declare_parameter("danger_distance_m", 0.22)
        self.declare_parameter("clear_distance_m", 0.35)

        self.declare_parameter("turn_speed", 0.45)
        self.declare_parameter("forward_speed", 0.07)

        self.declare_parameter("turn_away_sec", 1.05)
        self.declare_parameter("side_forward_sec", 1.20)
        self.declare_parameter("return_turn_sec", 0.95)
        self.declare_parameter("pass_forward_sec", 0.90)

        self.declare_parameter("default_direction", -1.0)  # -1 right, +1 left
        self.declare_parameter("auto_choose_direction", True)

        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("log_period_sec", 1.0)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.request_topic = str(self.get_parameter("request_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.front_min_deg = float(self.get_parameter("front_min_deg").value)
        self.front_max_deg = float(self.get_parameter("front_max_deg").value)

        self.left_min_deg = float(self.get_parameter("left_min_deg").value)
        self.left_max_deg = float(self.get_parameter("left_max_deg").value)

        self.right_min_deg = float(self.get_parameter("right_min_deg").value)
        self.right_max_deg = float(self.get_parameter("right_max_deg").value)

        self.danger_distance_m = float(self.get_parameter("danger_distance_m").value)
        self.clear_distance_m = float(self.get_parameter("clear_distance_m").value)

        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)

        self.turn_away_sec = float(self.get_parameter("turn_away_sec").value)
        self.side_forward_sec = float(self.get_parameter("side_forward_sec").value)
        self.return_turn_sec = float(self.get_parameter("return_turn_sec").value)
        self.pass_forward_sec = float(self.get_parameter("pass_forward_sec").value)

        self.default_direction = float(self.get_parameter("default_direction").value)
        self.default_direction = 1.0 if self.default_direction >= 0.0 else -1.0

        self.auto_choose_direction = bool(
            self.get_parameter("auto_choose_direction").value
        )

        self.scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        self.request_sub = self.create_subscription(
            String,
            self.request_topic,
            self.request_callback,
            10,
        )

        self.last_scan = None
        self.last_scan_t = 0.0

        self.front_min = float("inf")
        self.left_min = float("inf")
        self.right_min = float("inf")

        self.state = "IDLE"
        self.phase_t = time.monotonic()
        self.direction = self.default_direction
        self.last_log_t = 0.0
        self.done_t = 0.0

        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"✅ lidar_avoidance_node started | scan={self.scan_topic}, "
            f"cmd={self.cmd_topic}, status={self.status_topic}"
        )

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def publish_status(self):
        active = self.state not in ("IDLE", "DONE", "NO_SCAN")

        data = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "state": self.state,
            "active": active,
            "done": self.state == "DONE",
            "direction": self.direction,
            "front_min": self.front_min if math.isfinite(self.front_min) else None,
            "left_min": self.left_min if math.isfinite(self.left_min) else None,
            "right_min": self.right_min if math.isfinite(self.right_min) else None,
            "scan_fresh": self.scan_is_fresh(),
        }

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.status_pub.publish(msg)

    def log(self, text):
        now = time.monotonic()
        if now - self.last_log_t >= self.log_period_sec:
            self.get_logger().info(text)
            self.last_log_t = now

    def scan_is_fresh(self):
        return (time.monotonic() - self.last_scan_t) <= self.scan_timeout_sec

    @staticmethod
    def normalize_deg(angle_deg):
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def sector_min(self, scan: LaserScan, min_deg, max_deg):
        if scan is None:
            return float("inf")

        best = float("inf")
        angle = scan.angle_min

        for r in scan.ranges:
            deg = self.normalize_deg(math.degrees(angle))

            if min_deg <= deg <= max_deg:
                if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                    best = min(best, float(r))

            angle += scan.angle_increment

        return best

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_t = time.monotonic()

        self.front_min = self.sector_min(
            msg,
            self.front_min_deg,
            self.front_max_deg,
        )
        self.left_min = self.sector_min(
            msg,
            self.left_min_deg,
            self.left_max_deg,
        )
        self.right_min = self.sector_min(
            msg,
            self.right_min_deg,
            self.right_max_deg,
        )

    def choose_direction(self):
        if not self.auto_choose_direction:
            return self.default_direction

        left = self.left_min if math.isfinite(self.left_min) else 0.0
        right = self.right_min if math.isfinite(self.right_min) else 0.0

        # angular.z > 0 — поворот вліво.
        # angular.z < 0 — поворот вправо.
        if left > right:
            return 1.0
        return -1.0

    def request_callback(self, msg: String):
        command = msg.data.strip().lower()

        if command == "start":
            if not self.scan_is_fresh():
                self.state = "NO_SCAN"
                self.publish_cmd(0.0, 0.0)
                self.publish_status()
                self.get_logger().warn("Avoidance requested, but /scan is not fresh")
                return

            if self.state not in ("IDLE", "DONE", "NO_SCAN"):
                return

            self.direction = self.choose_direction()
            self.state = "TURN_AWAY"
            self.phase_t = time.monotonic()
            self.done_t = 0.0

            self.get_logger().info(
                f"🚧 AVOIDANCE START | direction="
                f"{'LEFT' if self.direction > 0 else 'RIGHT'} | "
                f"front={self.front_min:.3f}, left={self.left_min:.3f}, "
                f"right={self.right_min:.3f}"
            )

        elif command == "reset":
            self.state = "IDLE"
            self.phase_t = time.monotonic()
            self.done_t = 0.0
            self.publish_cmd(0.0, 0.0)
            self.publish_status()
            self.get_logger().info("Avoidance reset")

        elif command == "stop":
            self.state = "IDLE"
            self.phase_t = time.monotonic()
            self.done_t = 0.0
            self.publish_cmd(0.0, 0.0)
            self.publish_status()
            self.get_logger().info("Avoidance stopped")

    def finish_done(self):
        self.state = "DONE"
        self.done_t = time.monotonic()
        self.publish_cmd(0.0, 0.0)
        self.publish_status()
        self.get_logger().info("✅ AVOIDANCE DONE")

    def control_loop(self):
        now = time.monotonic()

        if self.state in ("IDLE", "DONE", "NO_SCAN"):
            self.publish_cmd(0.0, 0.0)
            self.publish_status()
            return

        if not self.scan_is_fresh():
            self.state = "NO_SCAN"
            self.publish_cmd(0.0, 0.0)
            self.publish_status()
            self.get_logger().warn("Avoidance stopped: /scan timeout")
            return

        elapsed = now - self.phase_t

        if self.state == "TURN_AWAY":
            self.publish_cmd(0.0, self.direction * self.turn_speed)
            self.publish_status()
            self.log(
                f"↪️ TURN_AWAY | dir={self.direction}, "
                f"front={self.front_min:.3f}"
            )

            if elapsed >= self.turn_away_sec:
                self.state = "SIDE_FORWARD"
                self.phase_t = now
                return

        elif self.state == "SIDE_FORWARD":
            # Якщо прямо знову небезпечно — ще трохи довертаємо.
            if self.front_min < self.danger_distance_m:
                self.publish_cmd(0.0, self.direction * self.turn_speed)
                self.publish_status()
                self.log(
                    f"⚠️ SIDE_FORWARD blocked | front={self.front_min:.3f}, turning more"
                )
                return

            self.publish_cmd(self.forward_speed, 0.0)
            self.publish_status()
            self.log(
                f"⬆️ SIDE_FORWARD | front={self.front_min:.3f}"
            )

            if elapsed >= self.side_forward_sec:
                self.state = "RETURN_TURN"
                self.phase_t = now
                return

        elif self.state == "RETURN_TURN":
            self.publish_cmd(0.0, -self.direction * self.turn_speed)
            self.publish_status()
            self.log("↩️ RETURN_TURN")

            if elapsed >= self.return_turn_sec:
                self.state = "PASS_FORWARD"
                self.phase_t = now
                return

        elif self.state == "PASS_FORWARD":
            if self.front_min < self.danger_distance_m:
                self.publish_cmd(0.0, 0.0)
                self.publish_status()
                self.log(
                    f"🛑 PASS_FORWARD blocked | front={self.front_min:.3f}"
                )
                return

            self.publish_cmd(self.forward_speed, 0.0)
            self.publish_status()
            self.log("⬆️ PASS_FORWARD")

            if elapsed >= self.pass_forward_sec:
                self.finish_done()
                return


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()