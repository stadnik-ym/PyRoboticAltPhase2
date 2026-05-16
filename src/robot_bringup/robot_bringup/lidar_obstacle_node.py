#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class State(Enum):
    CRUISE = "CRUISE"          # їдемо прямо
    STOP = "STOP"              # коротка зупинка перед маневром
    TURN_AWAY = "TURN_AWAY"    # повертаємо від перешкоди
    BYPASS = "BYPASS"          # об'їжджаємо перешкоду
    REALIGN = "REALIGN"        # повертаємось на початковий напрямок


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    """Нормалізація кута в радіанах до [-pi; pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_diff(target, current):
    """Коротка різниця target - current."""
    return normalize_angle(target - current)


def angle_diff_deg(a, b):
    """Коротка різниця a - b у градусах."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def quaternion_to_yaw(q):
    """Quaternion -> yaw без tf_transformations."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node")

        # ---------------- Topics ----------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")

        # Якщо є /odom — нода точніше повертається у старий напрямок
        self.declare_parameter("use_odom", True)

        # ---------------- Speeds ----------------
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("forward_speed", 0.18)
        self.declare_parameter("bypass_speed", 0.13)
        self.declare_parameter("turn_speed", 0.45)
        self.declare_parameter("max_angular_speed", 0.65)
        self.declare_parameter("min_turn_speed", 0.20)

        # ---------------- Distances ----------------
        self.declare_parameter("stop_distance", 0.42)
        self.declare_parameter("clear_distance", 0.62)
        self.declare_parameter("side_target_distance", 0.45)
        self.declare_parameter("side_lost_distance", 0.85)
        self.declare_parameter("scan_timeout", 0.7)

        # ---------------- LiDAR sectors ----------------
        # 0 градусів — напрямок вперед.
        # Якщо у твоєму /scan перед роботом не 0°, зміни front_angle_deg.
        self.declare_parameter("front_angle_deg", 0.0)
        self.declare_parameter("front_width_deg", 40.0)
        self.declare_parameter("diag_angle_deg", 45.0)
        self.declare_parameter("diag_width_deg", 55.0)
        self.declare_parameter("side_width_deg", 60.0)

        # ---------------- FSM timings ----------------
        self.declare_parameter("stop_time", 0.35)
        self.declare_parameter("turn_away_angle_deg", 35.0)
        self.declare_parameter("turn_away_time_no_odom", 1.0)
        self.declare_parameter("return_turn_time_no_odom", 1.0)
        self.declare_parameter("yaw_tolerance_deg", 8.0)
        self.declare_parameter("side_lost_time", 0.60)
        self.declare_parameter("min_bypass_time", 0.80)
        self.declare_parameter("max_bypass_time", 12.0)

        # ---------------- Control gains ----------------
        self.declare_parameter("wall_kp", 1.15)
        self.declare_parameter("front_avoid_kp", 0.9)
        self.declare_parameter("realign_kp", 1.8)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.use_odom = bool(self.get_parameter("use_odom").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.bypass_speed = float(self.get_parameter("bypass_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.min_turn_speed = float(self.get_parameter("min_turn_speed").value)

        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.clear_distance = float(self.get_parameter("clear_distance").value)
        self.side_target_distance = float(self.get_parameter("side_target_distance").value)
        self.side_lost_distance = float(self.get_parameter("side_lost_distance").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)

        self.front_angle_deg = float(self.get_parameter("front_angle_deg").value)
        self.front_width_deg = float(self.get_parameter("front_width_deg").value)
        self.diag_angle_deg = float(self.get_parameter("diag_angle_deg").value)
        self.diag_width_deg = float(self.get_parameter("diag_width_deg").value)
        self.side_width_deg = float(self.get_parameter("side_width_deg").value)

        self.stop_time = float(self.get_parameter("stop_time").value)
        self.turn_away_angle = math.radians(
            float(self.get_parameter("turn_away_angle_deg").value)
        )
        self.turn_away_time_no_odom = float(
            self.get_parameter("turn_away_time_no_odom").value
        )
        self.return_turn_time_no_odom = float(
            self.get_parameter("return_turn_time_no_odom").value
        )
        self.yaw_tolerance = math.radians(
            float(self.get_parameter("yaw_tolerance_deg").value)
        )
        self.side_lost_time = float(self.get_parameter("side_lost_time").value)
        self.min_bypass_time = float(self.get_parameter("min_bypass_time").value)
        self.max_bypass_time = float(self.get_parameter("max_bypass_time").value)

        self.wall_kp = float(self.get_parameter("wall_kp").value)
        self.front_avoid_kp = float(self.get_parameter("front_avoid_kp").value)
        self.realign_kp = float(self.get_parameter("realign_kp").value)

        # ---------------- State memory ----------------
        self.state = State.CRUISE
        self.state_enter_time = self.now()

        self.last_scan_time = None
        self.scan_data = {
            "front": float("inf"),
            "front_left": float("inf"),
            "front_right": float("inf"),
            "left": float("inf"),
            "right": float("inf"),
        }

        self.have_odom = False
        self.current_yaw = 0.0
        self.base_yaw = None

        # side: "right" означає, що робот об'їжджає перешкоду справа.
        # Тобто спочатку повертає вправо, а сама перешкода буде зліва від робота.
        self.bypass_side = "right"

        self.seen_side_obstacle = False
        self.side_lost_since = None
        self.last_warn_time = None

        # ---------------- ROS interfaces ----------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info(
            f"Obstacle avoidance started. scan={self.scan_topic}, "
            f"odom={self.odom_topic}, cmd={self.cmd_vel_topic}, use_odom={self.use_odom}"
        )

    def now(self):
        return self.get_clock().now()

    def seconds_since(self, stamp):
        return (self.now() - stamp).nanoseconds / 1e9

    def state_elapsed(self):
        return self.seconds_since(self.state_enter_time)

    def change_state(self, new_state):
        if new_state == self.state:
            return

        self.get_logger().info(
            f"{self.state.value} -> {new_state.value} | "
            f"front={self.scan_data['front']:.3f} m | side={self.bypass_side}"
        )

        self.state = new_state
        self.state_enter_time = self.now()

        if new_state == State.BYPASS:
            self.seen_side_obstacle = False
            self.side_lost_since = None

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------

    def odom_callback(self, msg):
        self.have_odom = True
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        self.last_scan_time = self.now()

        sector_defs = {
            "front": (
                self.front_angle_deg,
                self.front_width_deg,
            ),
            "front_left": (
                self.front_angle_deg + self.diag_angle_deg,
                self.diag_width_deg,
            ),
            "front_right": (
                self.front_angle_deg - self.diag_angle_deg,
                self.diag_width_deg,
            ),
            "left": (
                self.front_angle_deg + 90.0,
                self.side_width_deg,
            ),
            "right": (
                self.front_angle_deg - 90.0,
                self.side_width_deg,
            ),
        }

        values = {
            "front": float("inf"),
            "front_left": float("inf"),
            "front_right": float("inf"),
            "left": float("inf"),
            "right": float("inf"),
        }

        r_min = max(float(msg.range_min), 0.02)
        r_max = float(msg.range_max)

        if not math.isfinite(r_max) or r_max <= r_min:
            r_max = 30.0

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            r = float(r)

            if r < r_min or r > r_max:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(normalize_angle(angle_rad))

            for name, (center_deg, width_deg) in sector_defs.items():
                half_width = width_deg * 0.5
                diff = abs(angle_diff_deg(angle_deg, center_deg))

                if diff <= half_width and r < values[name]:
                    values[name] = r

        self.scan_data = values

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------

    def scan_is_fresh(self):
        if self.last_scan_time is None:
            return False
        return self.seconds_since(self.last_scan_time) <= self.scan_timeout

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(clamp(angular_z, -self.max_angular_speed, self.max_angular_speed))
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    def capped_distance(self, value):
        if not math.isfinite(value):
            return self.clear_distance * 2.0
        return min(value, self.clear_distance * 2.0)

    def choose_bypass_side(self):
        left_score = (
            self.capped_distance(self.scan_data["front_left"])
            + 0.7 * self.capped_distance(self.scan_data["left"])
        )

        right_score = (
            self.capped_distance(self.scan_data["front_right"])
            + 0.7 * self.capped_distance(self.scan_data["right"])
        )

        if right_score >= left_score:
            self.bypass_side = "right"
        else:
            self.bypass_side = "left"

        self.get_logger().info(
            f"Chosen bypass side: {self.bypass_side} | "
            f"left_score={left_score:.2f}, right_score={right_score:.2f}"
        )

    def turn_sign_away(self):
        # right => повертаємо вправо => angular.z negative
        if self.bypass_side == "right":
            return -1.0
        return 1.0

    def turn_sign_back(self):
        # Якщо спочатку повернули вправо, назад повертаємо вліво
        return -self.turn_sign_away()

    # ------------------------------------------------------------
    # Main FSM
    # ------------------------------------------------------------

    def control_loop(self):
        if not self.scan_is_fresh():
            self.publish_stop()

            if self.last_warn_time is None or self.seconds_since(self.last_warn_time) > 1.0:
                self.get_logger().warn("No fresh /scan data. Robot stopped.")
                self.last_warn_time = self.now()

            return

        if self.state == State.CRUISE:
            self.handle_cruise()
        elif self.state == State.STOP:
            self.handle_stop()
        elif self.state == State.TURN_AWAY:
            self.handle_turn_away()
        elif self.state == State.BYPASS:
            self.handle_bypass()
        elif self.state == State.REALIGN:
            self.handle_realign()
        else:
            self.publish_stop()

    def handle_cruise(self):
        front = self.scan_data["front"]

        if front <= self.stop_distance:
            self.publish_stop()

            if self.use_odom and self.have_odom:
                self.base_yaw = self.current_yaw
            else:
                self.base_yaw = None

            self.choose_bypass_side()
            self.change_state(State.STOP)
            return

        self.publish_cmd(self.forward_speed, 0.0)

    def handle_stop(self):
        self.publish_stop()

        if self.state_elapsed() >= self.stop_time:
            self.choose_bypass_side()
            self.change_state(State.TURN_AWAY)

    def handle_turn_away(self):
        front = self.scan_data["front"]

        angular = self.turn_sign_away() * self.turn_speed
        self.publish_cmd(0.0, angular)

        front_clear = front >= self.clear_distance

        if self.use_odom and self.have_odom and self.base_yaw is not None:
            turned_enough = abs(angle_diff(self.current_yaw, self.base_yaw)) >= self.turn_away_angle
        else:
            turned_enough = self.state_elapsed() >= self.turn_away_time_no_odom

        if front_clear and turned_enough:
            self.change_state(State.BYPASS)

    def handle_bypass(self):
        front = self.scan_data["front"]

        # Якщо об'їжджаємо справа, перешкода має бути зліва.
        # Якщо об'їжджаємо зліва, перешкода має бути справа.
        if self.bypass_side == "right":
            wall_key = "left"
            wall_sign = 1.0
        else:
            wall_key = "right"
            wall_sign = -1.0

        side_dist = self.scan_data[wall_key]

        # Визначаємо, що ми вже дійсно проїжджали біля перешкоди збоку
        if side_dist <= self.side_lost_distance:
            self.seen_side_obstacle = True
            self.side_lost_since = None

        # Коли перешкода збоку зникла — значить, скоріше за все, ми її об'їхали
        if (
            self.seen_side_obstacle
            and side_dist > self.side_lost_distance
            and front > self.clear_distance
            and self.state_elapsed() >= self.min_bypass_time
        ):
            if self.side_lost_since is None:
                self.side_lost_since = self.now()
            elif self.seconds_since(self.side_lost_since) >= self.side_lost_time:
                self.publish_stop()
                self.change_state(State.REALIGN)
                return
        else:
            if side_dist <= self.side_lost_distance:
                self.side_lost_since = None

        # Захист від зависання у маневрі
        if self.state_elapsed() >= self.max_bypass_time:
            self.get_logger().warn("Bypass timeout. Trying to realign.")
            self.publish_stop()
            self.change_state(State.REALIGN)
            return

        # Якщо прямо знову близько — сильніше повертаємо від перешкоди
        if front <= self.stop_distance:
            linear = 0.03
            angular = self.turn_sign_away() * self.turn_speed
            self.publish_cmd(linear, angular)
            return

        # Wall-following: тримаємо приблизну бокову дистанцію
        if math.isfinite(side_dist):
            error = side_dist - self.side_target_distance
        else:
            error = 0.0

        angular = wall_sign * self.wall_kp * error

        # Додаткове відвертання, якщо попереду ще не повністю чисто
        if front < self.clear_distance:
            danger = (self.clear_distance - front) / max(
                0.01,
                self.clear_distance - self.stop_distance,
            )
            angular += self.turn_sign_away() * self.front_avoid_kp * danger

        angular = clamp(
            angular,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        self.publish_cmd(self.bypass_speed, angular)

    def handle_realign(self):
        front = self.scan_data["front"]

        # Якщо під час повернення на курс знову щось перед нами —
        # продовжуємо об'їзд.
        if front <= self.stop_distance:
            self.change_state(State.TURN_AWAY)
            return

        if self.use_odom and self.have_odom and self.base_yaw is not None:
            diff = angle_diff(self.base_yaw, self.current_yaw)

            if abs(diff) <= self.yaw_tolerance:
                self.get_logger().info("Realigned. Continue cruise.")
                self.change_state(State.CRUISE)
                self.publish_cmd(self.forward_speed, 0.0)
                return

            angular = self.realign_kp * diff

            if abs(angular) < self.min_turn_speed:
                angular = math.copysign(self.min_turn_speed, angular)

            angular = clamp(
                angular,
                -self.turn_speed,
                self.turn_speed,
            )

            self.publish_cmd(0.0, angular)
            return

        # Fallback без /odom: повертаємося приблизно по часу
        if self.state_elapsed() < self.return_turn_time_no_odom:
            angular = self.turn_sign_back() * self.turn_speed
            self.publish_cmd(0.0, angular)
        else:
            self.get_logger().info("Approximate realign without odom. Continue cruise.")
            self.change_state(State.CRUISE)
            self.publish_cmd(self.forward_speed, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()