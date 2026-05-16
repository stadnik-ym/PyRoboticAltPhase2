#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32


class MissionMasterNode(Node):
    def __init__(self):
        super().__init__("mission_master")

        # ============================================================
        # TOPICS
        # ============================================================
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("buzzer_topic", "/buzzer_signal")
        self.declare_parameter("aruco_id_topic", "/aruco/id")
        self.declare_parameter("aruco_offset_topic", "/aruco/offset_px")
        self.declare_parameter("aruco_width_topic", "/aruco/width")

        # ============================================================
        # MAIN LOOP / DEBUG
        # ============================================================
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("enable_aruco_debug_log", True)
        self.declare_parameter("debug_log_period_sec", 1.0)
        self.declare_parameter("enable_cmd_debug_log", True)

        # ============================================================
        # ARUCO SETTINGS
        # ============================================================
        self.declare_parameter("aruco_timeout", 0.6)
        self.declare_parameter("aruco_lost_grace_sec", 1.2)
        self.declare_parameter("target_marker_ids", "")
        self.declare_parameter("ignore_passed_markers", True)
        self.declare_parameter("finish_after_first_marker", False)

        # ============================================================
        # SECTOR SEARCH SETTINGS
        # ============================================================
        self.declare_parameter("sector_angle_deg", 20.0)
        self.declare_parameter("sector_pause_sec", 2.0)
        self.declare_parameter("sector_turn_speed", 0.8)
        self.declare_parameter("sector_turn_duration_sec", 0.0)
        self.declare_parameter("search_direction", 1.0)
        self.declare_parameter("max_sector_count", 18)

        # ============================================================
        # TRACKING SETTINGS
        # ============================================================
        # marker_stop_width — це ширина ArUco в пікселях, не мм.
        # Для запуску об'їзду на ~10 см підбери значення через:
        # ros2 topic echo /aruco/width
        self.declare_parameter("marker_stop_width", 420.0)
        self.declare_parameter("center_deadzone_px", 80.0)
        self.declare_parameter("aruco_turn_kp", 0.0025)
        self.declare_parameter("max_track_angular", 0.30)
        self.declare_parameter("approach_speed", 0.05)
        self.declare_parameter("offset_to_angular_sign", -1.0)
        self.declare_parameter("allow_forward_while_turning", True)
        self.declare_parameter("forward_turn_offset_limit_px", 320.0)

        # ============================================================
        # AVOIDANCE SETTINGS
        # ============================================================
        self.declare_parameter("enable_avoidance", True)
        self.declare_parameter("avoid_direction", -1.0)  # -1.0 вправо, +1.0 вліво
        self.declare_parameter("avoid_turn_speed", 0.7)
        self.declare_parameter("avoid_turn_duration_sec", 1.1)
        self.declare_parameter("avoid_forward_speed", 0.07)
        self.declare_parameter("avoid_forward_duration_sec", 1.3)
        self.declare_parameter("avoid_return_turn_duration_sec", 1.1)
        self.declare_parameter("avoid_pass_forward_duration_sec", 0.9)
        self.declare_parameter("avoid_pause_between_phases_sec", 0.15)

        # ============================================================
        # BUZZER / MARKER REACHED
        # ============================================================
        self.declare_parameter("buzzer_on_value", 1)
        self.declare_parameter("buzzer_off_value", 0)
        self.declare_parameter("beep_duration_sec", 0.35)
        self.declare_parameter("reached_pause_sec", 1.5)

        # ============================================================
        # OPTIONAL FRONT GUARD
        # ============================================================
        self.declare_parameter("use_front_distance_guard", False)
        self.declare_parameter("front_distance_topic", "/lidar/front_distance")
        self.declare_parameter("front_stop_distance", 0.25)

        # ============================================================
        # READ PARAMETERS
        # ============================================================
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.buzzer_topic = self.get_parameter("buzzer_topic").value
        self.aruco_id_topic = self.get_parameter("aruco_id_topic").value
        self.aruco_offset_topic = self.get_parameter("aruco_offset_topic").value
        self.aruco_width_topic = self.get_parameter("aruco_width_topic").value

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.enable_aruco_debug_log = bool(self.get_parameter("enable_aruco_debug_log").value)
        self.debug_log_period_sec = float(self.get_parameter("debug_log_period_sec").value)
        self.enable_cmd_debug_log = bool(self.get_parameter("enable_cmd_debug_log").value)

        self.aruco_timeout = float(self.get_parameter("aruco_timeout").value)
        self.aruco_lost_grace_sec = float(self.get_parameter("aruco_lost_grace_sec").value)
        self.target_marker_ids = self.parse_marker_ids(self.get_parameter("target_marker_ids").value)
        self.ignore_passed_markers = bool(self.get_parameter("ignore_passed_markers").value)
        self.finish_after_first_marker = bool(self.get_parameter("finish_after_first_marker").value)

        self.sector_angle_deg = float(self.get_parameter("sector_angle_deg").value)
        self.sector_pause_sec = float(self.get_parameter("sector_pause_sec").value)
        self.sector_turn_speed = abs(float(self.get_parameter("sector_turn_speed").value))
        self.sector_turn_duration_sec = float(self.get_parameter("sector_turn_duration_sec").value)
        self.search_direction = float(self.get_parameter("search_direction").value)
        self.max_sector_count = int(self.get_parameter("max_sector_count").value)

        self.search_direction = 1.0 if self.search_direction >= 0.0 else -1.0
        if self.sector_turn_speed <= 0.0:
            self.sector_turn_speed = 0.8
        if self.sector_turn_duration_sec <= 0.0:
            self.sector_turn_duration_sec = math.radians(self.sector_angle_deg) / self.sector_turn_speed

        self.marker_stop_width = float(self.get_parameter("marker_stop_width").value)
        self.center_deadzone_px = float(self.get_parameter("center_deadzone_px").value)
        self.aruco_turn_kp = float(self.get_parameter("aruco_turn_kp").value)
        self.max_track_angular = abs(float(self.get_parameter("max_track_angular").value))
        self.approach_speed = float(self.get_parameter("approach_speed").value)
        self.offset_to_angular_sign = float(self.get_parameter("offset_to_angular_sign").value)
        self.allow_forward_while_turning = bool(self.get_parameter("allow_forward_while_turning").value)
        self.forward_turn_offset_limit_px = float(self.get_parameter("forward_turn_offset_limit_px").value)

        self.enable_avoidance = bool(self.get_parameter("enable_avoidance").value)
        self.avoid_direction = float(self.get_parameter("avoid_direction").value)
        self.avoid_direction = 1.0 if self.avoid_direction >= 0.0 else -1.0
        self.avoid_turn_speed = abs(float(self.get_parameter("avoid_turn_speed").value))
        self.avoid_turn_duration_sec = float(self.get_parameter("avoid_turn_duration_sec").value)
        self.avoid_forward_speed = float(self.get_parameter("avoid_forward_speed").value)
        self.avoid_forward_duration_sec = float(self.get_parameter("avoid_forward_duration_sec").value)
        self.avoid_return_turn_duration_sec = float(self.get_parameter("avoid_return_turn_duration_sec").value)
        self.avoid_pass_forward_duration_sec = float(self.get_parameter("avoid_pass_forward_duration_sec").value)
        self.avoid_pause_between_phases_sec = float(self.get_parameter("avoid_pause_between_phases_sec").value)

        self.buzzer_on_value = int(self.get_parameter("buzzer_on_value").value)
        self.buzzer_off_value = int(self.get_parameter("buzzer_off_value").value)
        self.beep_duration_sec = float(self.get_parameter("beep_duration_sec").value)
        self.reached_pause_sec = float(self.get_parameter("reached_pause_sec").value)

        self.use_front_distance_guard = bool(self.get_parameter("use_front_distance_guard").value)
        self.front_distance_topic = self.get_parameter("front_distance_topic").value
        self.front_stop_distance = float(self.get_parameter("front_stop_distance").value)

        # ============================================================
        # INTERNAL STATE
        # ============================================================
        self.state = "SEARCH_ARUCO"

        self.search_phase = "WAIT"
        self.search_phase_start_time = self.now_sec()
        self.current_sector_index = 0

        self.aruco_id = -1
        self.aruco_offset_px = 0.0
        self.aruco_width = 0.0
        self.last_aruco_time = -999.0

        self.last_raw_aruco_id = -1
        self.vision_reports_marker = False

        self.active_marker_id = -1
        self.passed_markers = set()

        self.marker_reached_start_time = 0.0
        self.buzzer_until_time = 0.0
        self.buzzer_is_on = False

        self.avoid_phase = "IDLE"
        self.avoid_phase_start_time = 0.0
        self.avoid_pause_until_time = 0.0

        self.front_distance = float("inf")
        self.last_front_distance_time = -999.0

        # Debug state
        self.aruco_id_msg_received = False
        self.aruco_offset_msg_received = False
        self.aruco_width_msg_received = False
        self.last_logged_aruco_id = None
        self.last_aruco_id_log_time = 0.0
        self.last_aruco_offset_log_time = 0.0
        self.last_aruco_width_log_time = 0.0
        self.last_search_status_log_time = 0.0
        self.last_cmd_log_time = 0.0
        self.last_track_log_time = 0.0

        # ============================================================
        # ROS PUB/SUB
        # ============================================================
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.buzzer_pub = self.create_publisher(Int32, self.buzzer_topic, 10)

        self.create_subscription(Int32, self.aruco_id_topic, self.aruco_id_callback, 10)
        self.create_subscription(Float32, self.aruco_offset_topic, self.aruco_offset_callback, 10)
        self.create_subscription(Float32, self.aruco_width_topic, self.aruco_width_callback, 10)

        if self.use_front_distance_guard:
            self.create_subscription(Float32, self.front_distance_topic, self.front_distance_callback, 10)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        self.reset_sector_search()

        self.get_logger().info("✅ MissionMasterNode запущено")
        self.get_logger().info(f"➡️ cmd_topic: {self.cmd_topic}")
        self.get_logger().info(f"🔔 buzzer_topic: {self.buzzer_topic}")
        self.get_logger().info(
            f"👁️ ArUco topics: id={self.aruco_id_topic}, "
            f"offset={self.aruco_offset_topic}, width={self.aruco_width_topic}"
        )
        self.get_logger().info(
            f"🔎 Секторальний пошук: {self.sector_angle_deg:.1f}° | "
            f"пауза {self.sector_pause_sec:.1f} c | "
            f"швидкість повороту {self.sector_turn_speed:.2f} rad/s | "
            f"тривалість повороту {self.sector_turn_duration_sec:.2f} c"
        )
        self.get_logger().info(
            f"🎯 Tracking: stop_width={self.marker_stop_width:.1f}px | "
            f"deadzone={self.center_deadzone_px:.1f}px | kp={self.aruco_turn_kp:.4f} | "
            f"forward_while_turning={self.allow_forward_while_turning}"
        )
        self.get_logger().info(
            f"🚧 Avoidance: enabled={self.enable_avoidance} | "
            f"direction={self.avoid_direction:+.0f} | turn={self.avoid_turn_duration_sec:.2f}s | "
            f"side_forward={self.avoid_forward_duration_sec:.2f}s"
        )
        if len(self.target_marker_ids) == 0:
            self.get_logger().info("🎯 Target ArUco: будь-який ID")
        else:
            self.get_logger().info(f"🎯 Target ArUco IDs: {self.target_marker_ids}")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def aruco_id_callback(self, msg: Int32):
        now = self.now_sec()
        self.aruco_id_msg_received = True

        raw_id = int(msg.data)
        self.last_raw_aruco_id = raw_id
        self.vision_reports_marker = raw_id >= 0

        if raw_id >= 0:
            self.aruco_id = raw_id
            self.last_aruco_time = now
        else:
            # Не скидаємо ID одразу. Даємо камері aruco_lost_grace_sec секунд
            # підхопити маркер знову.
            if (now - self.last_aruco_time) > self.aruco_lost_grace_sec:
                self.aruco_id = -1
                self.aruco_offset_px = 0.0
                self.aruco_width = 0.0

        should_log = self.last_logged_aruco_id != raw_id
        if (now - self.last_aruco_id_log_time) >= self.debug_log_period_sec:
            should_log = True

        if self.enable_aruco_debug_log and should_log:
            self.get_logger().info(
                f"👁️ RX {self.aruco_id_topic}: raw_id={raw_id}, "
                f"stored_id={self.aruco_id}, vision_valid={self.vision_reports_marker}"
            )
            self.last_aruco_id_log_time = now
            self.last_logged_aruco_id = raw_id

    def aruco_offset_callback(self, msg: Float32):
        now = self.now_sec()
        self.aruco_offset_msg_received = True
        raw_offset = float(msg.data)

        # Якщо vision зараз бачить маркер — оновлюємо offset.
        # Якщо прийшов id=-1, offset=0.0 не стирає останній корисний offset.
        if self.vision_reports_marker:
            self.aruco_offset_px = raw_offset
            self.last_aruco_time = now

        if (
            self.enable_aruco_debug_log
            and (now - self.last_aruco_offset_log_time) >= self.debug_log_period_sec
        ):
            self.get_logger().info(
                f"👁️ RX {self.aruco_offset_topic}: "
                f"raw_offset={raw_offset:.1f}, stored_offset={self.aruco_offset_px:.1f}"
            )
            self.last_aruco_offset_log_time = now

    def aruco_width_callback(self, msg: Float32):
        now = self.now_sec()
        self.aruco_width_msg_received = True
        raw_width = float(msg.data)

        # Не стираємо width=0.0 від невдалого кадру, якщо ще не вийшов grace-time.
        if self.vision_reports_marker:
            self.aruco_width = raw_width
            self.last_aruco_time = now

        if (
            self.enable_aruco_debug_log
            and (now - self.last_aruco_width_log_time) >= self.debug_log_period_sec
        ):
            self.get_logger().info(
                f"👁️ RX {self.aruco_width_topic}: "
                f"raw_width={raw_width:.1f}, stored_width={self.aruco_width:.1f}"
            )
            self.last_aruco_width_log_time = now

    def front_distance_callback(self, msg: Float32):
        self.front_distance = float(msg.data)
        self.last_front_distance_time = self.now_sec()

    # ============================================================
    # UTILS
    # ============================================================

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def parse_marker_ids(self, value):
        if value is None:
            return []
        text = str(value).strip()
        if text == "":
            return []
        result = []
        for part in text.split(","):
            part = part.strip()
            if part == "":
                continue
            try:
                result.append(int(part))
            except ValueError:
                self.get_logger().warn(f"⚠️ Некоректний ArUco ID у target_marker_ids: '{part}'")
        return result

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def publish_cmd(self, linear_x, angular_z):
        if self.use_front_distance_guard:
            front_is_fresh = (self.now_sec() - self.last_front_distance_time) < 0.5
            if front_is_fresh and linear_x > 0.0 and self.front_distance <= self.front_stop_distance:
                self.get_logger().warn(
                    f"🛑 FRONT GUARD: front={self.front_distance:.3f} m, "
                    f"stop={self.front_stop_distance:.3f} m. linear_x forced to 0."
                )
                linear_x = 0.0

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

        now = self.now_sec()
        if (
            self.enable_cmd_debug_log
            and (abs(linear_x) > 0.001 or abs(angular_z) > 0.001)
            and (now - self.last_cmd_log_time) >= 0.5
        ):
            self.get_logger().info(
                f"📤 CMD -> {self.cmd_topic}: linear={linear_x:.3f}, angular={angular_z:.3f}"
            )
            self.last_cmd_log_time = now

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def publish_buzzer(self, value):
        msg = Int32()
        msg.data = int(value)
        self.buzzer_pub.publish(msg)

    def buzzer_on(self):
        self.publish_buzzer(self.buzzer_on_value)
        self.buzzer_is_on = True

    def buzzer_off(self):
        self.publish_buzzer(self.buzzer_off_value)
        self.buzzer_is_on = False

    def set_state(self, new_state):
        if self.state == new_state:
            return
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f"🔁 STATE: {old_state} -> {new_state}")
        if new_state == "SEARCH_ARUCO":
            self.reset_sector_search()

    def reset_sector_search(self):
        self.search_phase = "WAIT"
        self.search_phase_start_time = self.now_sec()
        self.current_sector_index = 0
        self.last_search_status_log_time = 0.0
        self.stop_robot()
        self.get_logger().info("🔎 Секторальний пошук ArUco запущено")

    def is_target_marker(self, marker_id):
        if marker_id < 0:
            return False
        if len(self.target_marker_ids) == 0:
            return True
        return marker_id in self.target_marker_ids

    def is_marker_passed(self, marker_id):
        if not self.ignore_passed_markers:
            return False
        return marker_id in self.passed_markers

    def has_active_aruco(self):
        now = self.now_sec()
        if self.aruco_id < 0:
            return False

        # У TRACK_ARUCO дозволяємо довше не отримувати валідний кадр,
        # щоб один id=-1 не ламав сценарій.
        timeout = self.aruco_lost_grace_sec if self.state == "TRACK_ARUCO" else self.aruco_timeout
        if (now - self.last_aruco_time) > timeout:
            return False
        if not self.is_target_marker(self.aruco_id):
            return False
        if self.is_marker_passed(self.aruco_id):
            return False
        return True

    def all_target_markers_passed(self):
        if len(self.target_marker_ids) == 0:
            return False
        for marker_id in self.target_marker_ids:
            if marker_id not in self.passed_markers:
                return False
        return True

    def log_aruco_search_status(self):
        now = self.now_sec()
        if not self.enable_aruco_debug_log:
            return
        if (now - self.last_search_status_log_time) < self.debug_log_period_sec:
            return

        age_text = "never" if self.last_aruco_time < -900.0 else f"{now - self.last_aruco_time:.2f}s"
        active = self.has_active_aruco()
        self.get_logger().info(
            f"📊 ARUCO STATUS | "
            f"rx_id={self.aruco_id_msg_received}, "
            f"rx_offset={self.aruco_offset_msg_received}, "
            f"rx_width={self.aruco_width_msg_received} | "
            f"raw_id={self.last_raw_aruco_id}, "
            f"stored_id={self.aruco_id}, "
            f"offset={self.aruco_offset_px:.1f}, "
            f"width={self.aruco_width:.1f}, "
            f"age={age_text}, active={active}"
        )
        self.last_search_status_log_time = now

    # ============================================================
    # MAIN CONTROL LOOP
    # ============================================================

    def control_loop(self):
        if self.state == "SEARCH_ARUCO":
            self.sector_aruco_search_control()
            return
        if self.state == "TRACK_ARUCO":
            self.track_aruco_control()
            return
        if self.state == "AVOID_OBSTACLE":
            self.avoid_obstacle_control()
            return
        if self.state == "MARKER_REACHED":
            self.marker_reached_control()
            return
        if self.state == "FINISHED":
            self.stop_robot()
            if self.buzzer_is_on:
                self.buzzer_off()
            return

        self.get_logger().warn(f"⚠️ Невідомий state: {self.state}")
        self.stop_robot()
        self.set_state("SEARCH_ARUCO")

    # ============================================================
    # SECTOR SEARCH
    # ============================================================

    def sector_aruco_search_control(self):
        now = self.now_sec()
        elapsed = now - self.search_phase_start_time
        self.log_aruco_search_status()

        if self.has_active_aruco():
            self.active_marker_id = self.aruco_id
            self.stop_robot()
            self.get_logger().info(
                f"✅ ArUco знайдено в секторі {self.current_sector_index}: "
                f"ID={self.active_marker_id}, offset={self.aruco_offset_px:.1f}, "
                f"width={self.aruco_width:.1f}"
            )
            self.set_state("TRACK_ARUCO")
            return

        if self.search_phase == "WAIT":
            self.stop_robot()
            if elapsed >= self.sector_pause_sec:
                self.search_phase = "TURN"
                self.search_phase_start_time = now
                self.get_logger().info(
                    f"↪️ Маркер не знайдено. Поворот до наступного сектору "
                    f"на ~{self.sector_angle_deg:.0f}°"
                )
            return

        if self.search_phase == "TURN":
            angular = self.search_direction * self.sector_turn_speed
            self.publish_cmd(0.0, angular)
            if elapsed >= self.sector_turn_duration_sec:
                self.stop_robot()
                self.current_sector_index += 1
                if self.max_sector_count > 0 and self.current_sector_index >= self.max_sector_count:
                    self.current_sector_index = 0
                    self.get_logger().info("🔄 Повне коло пошуку завершено")
                self.search_phase = "WAIT"
                self.search_phase_start_time = now
                self.get_logger().info(f"⏸️ Зупинка для аналізу сектору {self.current_sector_index}")
            return

        self.get_logger().warn(f"⚠️ Невідома search_phase: {self.search_phase}. Reset пошуку.")
        self.reset_sector_search()

    # ============================================================
    # ARUCO TRACKING
    # ============================================================

    def track_aruco_control(self):
        now = self.now_sec()

        if not self.has_active_aruco():
            self.stop_robot()
            self.get_logger().warn(
                f"⚠️ ArUco втрачено. stored_id={self.aruco_id}, "
                f"raw_id={self.last_raw_aruco_id}, "
                f"last_age={now - self.last_aruco_time:.2f}s. Повернення до пошуку."
            )
            self.set_state("SEARCH_ARUCO")
            return

        self.active_marker_id = self.aruco_id

        # Якщо маркер вже достатньо близько — запускаємо об'їзд або фіксацію.
        if self.aruco_width >= self.marker_stop_width:
            self.stop_robot()
            self.get_logger().info(
                f"📍 ArUco ID={self.active_marker_id} на порозі дії | "
                f"width={self.aruco_width:.1f} >= trigger_width={self.marker_stop_width:.1f}"
            )
            if self.enable_avoidance:
                self.start_avoidance()
            else:
                self.start_marker_reached()
            return

        offset = self.aruco_offset_px

        if abs(offset) > self.center_deadzone_px:
            angular = self.offset_to_angular_sign * self.aruco_turn_kp * offset
            angular = self.clamp(angular, -self.max_track_angular, self.max_track_angular)

            if self.allow_forward_while_turning and abs(offset) <= self.forward_turn_offset_limit_px:
                linear = self.approach_speed * 0.45
            else:
                linear = 0.0

            self.publish_cmd(linear, angular)

            if now - self.last_track_log_time >= 0.35:
                self.get_logger().info(
                    f"🎯 Підрулювання до ArUco ID={self.active_marker_id} | "
                    f"offset={offset:.1f}px | width={self.aruco_width:.1f}px | "
                    f"linear={linear:.3f} | angular={angular:.3f}"
                )
                self.last_track_log_time = now
            return

        self.publish_cmd(self.approach_speed, 0.0)
        if now - self.last_track_log_time >= 0.35:
            self.get_logger().info(
                f"⬆️ Рух до ArUco ID={self.active_marker_id} | "
                f"offset={offset:.1f}px | width={self.aruco_width:.1f}/{self.marker_stop_width:.1f}"
            )
            self.last_track_log_time = now

    # ============================================================
    # AVOIDANCE
    # ============================================================

    def start_avoidance(self):
        now = self.now_sec()
        self.avoid_phase = "TURN_AWAY"
        self.avoid_phase_start_time = now
        self.avoid_pause_until_time = 0.0
        self.buzzer_until_time = now + self.beep_duration_sec
        self.buzzer_on()
        self.get_logger().info(
            f"🚧 Старт об'їзду ArUco ID={self.active_marker_id} | "
            f"direction={'left' if self.avoid_direction > 0 else 'right'}"
        )
        self.set_state("AVOID_OBSTACLE")

    def set_avoid_phase(self, phase_name):
        now = self.now_sec()
        self.stop_robot()
        self.avoid_phase = phase_name
        self.avoid_phase_start_time = now
        self.avoid_pause_until_time = now + self.avoid_pause_between_phases_sec

    def avoid_obstacle_control(self):
        now = self.now_sec()

        if self.buzzer_is_on and now >= self.buzzer_until_time:
            self.buzzer_off()

        if now < self.avoid_pause_until_time:
            self.stop_robot()
            return

        elapsed = now - max(self.avoid_phase_start_time, self.avoid_pause_until_time)

        if self.avoid_phase == "TURN_AWAY":
            self.publish_cmd(0.0, self.avoid_direction * self.avoid_turn_speed)
            if elapsed >= self.avoid_turn_duration_sec:
                self.get_logger().info("↪️ Об'їзд: поворот убік завершено")
                self.set_avoid_phase("FORWARD_SIDE")
            return

        if self.avoid_phase == "FORWARD_SIDE":
            self.publish_cmd(self.avoid_forward_speed, 0.0)
            if elapsed >= self.avoid_forward_duration_sec:
                self.get_logger().info("⬆️ Об'їзд: боковий проїзд завершено")
                self.set_avoid_phase("TURN_BACK")
            return

        if self.avoid_phase == "TURN_BACK":
            self.publish_cmd(0.0, -self.avoid_direction * self.avoid_turn_speed)
            if elapsed >= self.avoid_return_turn_duration_sec:
                self.get_logger().info("↩️ Об'їзд: повернення напрямку завершено")
                self.set_avoid_phase("PASS_FORWARD")
            return

        if self.avoid_phase == "PASS_FORWARD":
            self.publish_cmd(self.avoid_forward_speed, 0.0)
            if elapsed >= self.avoid_pass_forward_duration_sec:
                self.stop_robot()
                if self.ignore_passed_markers and self.active_marker_id >= 0:
                    self.passed_markers.add(self.active_marker_id)
                self.get_logger().info(f"✅ Об'їзд ArUco ID={self.active_marker_id} завершено")
                self.active_marker_id = -1
                self.avoid_phase = "IDLE"

                if self.finish_after_first_marker:
                    self.set_state("FINISHED")
                elif self.all_target_markers_passed():
                    self.set_state("FINISHED")
                else:
                    self.set_state("SEARCH_ARUCO")
            return

        self.get_logger().warn(f"⚠️ Невідома avoid_phase: {self.avoid_phase}. Повернення до пошуку.")
        self.stop_robot()
        self.avoid_phase = "IDLE"
        self.set_state("SEARCH_ARUCO")

    # ============================================================
    # MARKER REACHED
    # ============================================================

    def start_marker_reached(self):
        self.stop_robot()
        now = self.now_sec()
        self.marker_reached_start_time = now
        self.buzzer_until_time = now + self.beep_duration_sec
        self.buzzer_on()
        self.get_logger().info(
            f"📍 ArUco ID={self.active_marker_id} досягнуто | "
            f"width={self.aruco_width:.1f} >= stop_width={self.marker_stop_width:.1f}"
        )
        self.set_state("MARKER_REACHED")

    def marker_reached_control(self):
        now = self.now_sec()
        elapsed = now - self.marker_reached_start_time
        self.stop_robot()

        if self.buzzer_is_on and now >= self.buzzer_until_time:
            self.buzzer_off()

        if elapsed < self.reached_pause_sec:
            return

        if self.ignore_passed_markers and self.active_marker_id >= 0:
            self.passed_markers.add(self.active_marker_id)

        self.get_logger().info(f"✅ Сценарій для ArUco ID={self.active_marker_id} завершено")

        if self.finish_after_first_marker:
            self.get_logger().info("🏁 Місію завершено після першого маркера")
            self.set_state("FINISHED")
            return

        if self.all_target_markers_passed():
            self.get_logger().info("🏁 Усі цільові ArUco маркери пройдено")
            self.set_state("FINISHED")
            return

        self.active_marker_id = -1
        self.set_state("SEARCH_ARUCO")


def main(args=None):
    rclpy.init(args=args)
    node = MissionMasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.buzzer_off()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
