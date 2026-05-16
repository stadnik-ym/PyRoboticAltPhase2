#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


class MissionMaster(Node):
    def __init__(self):
        super().__init__("mission_master")

        # =========================================================
        # PARAMETERS
        # =========================================================

        self.declare_parameter("aruco_status_topic", "/aruco/status")
        self.declare_parameter("front_distance_topic", "/lidar/front_distance")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        self.declare_parameter("control_hz", 20.0)

        # ---------------- ARUCO TRACKING ----------------

        self.declare_parameter("aruco_lost_grace_sec", 3.0)

        self.declare_parameter("center_deadzone_px", 90.0)
        self.declare_parameter("aruco_turn_kp", 0.0010)
        self.declare_parameter("max_track_angular", 0.12)
        self.declare_parameter("approach_speed", 0.025)

        self.declare_parameter("allow_forward_while_turning", False)
        self.declare_parameter("forward_turn_offset_limit_px", 160.0)

        # Якщо робот повертає не в той бік під час центрування — постав True у launch.
        self.declare_parameter("invert_aruco_turn", False)

        # ---------------- LOST MARKER RECOVERY ----------------

        self.declare_parameter("assume_reached_if_lost_distance_m", 0.80)
        self.declare_parameter("assume_reached_if_lost_width_px", 80.0)

        # False — щоб робот не робив 180/360 по старому offset.
        self.declare_parameter("recover_turn_to_last_marker", False)
        self.declare_parameter("recover_turn_speed", 0.08)

        # ---------------- FORCE BYPASS TRIGGER ----------------

        self.declare_parameter("enable_force_bypass_trigger", True)

        # Стартуємо об'їзд раніше, щоб не впиратися в ArUco.
        self.declare_parameter("force_bypass_if_front_below_m", 0.75)
        self.declare_parameter("force_bypass_if_width_above_px", 85.0)
        self.declare_parameter("force_bypass_if_camera_est_below_m", 0.95)

        # Захист від старої ArUco після об'їзду.
        self.declare_parameter("force_reached_max_marker_age_sec", 4.0)

        # ---------------- STOP NEAR MARKER ----------------

        self.declare_parameter("use_lidar_for_marker_stop", True)

        # Починаємо об'їзд приблизно з 70 см.
        self.declare_parameter("target_marker_distance_m", 0.70)

        # Запасний тригер по ширині мітки.
        self.declare_parameter("marker_stop_width_px_override", 85.0)

        self.declare_parameter("marker_real_size_cm", 18.7)
        self.declare_parameter("camera_hfov_deg", 60.0)

        self.declare_parameter("emergency_reached_distance_m", 0.45)

        # ---------------- SEARCH ----------------

        self.declare_parameter("search_turn_speed", 0.12)
        self.declare_parameter("search_turn_duration_sec", 0.35)
        self.declare_parameter("search_pause_sec", 0.70)

        # ---------------- CURVE BYPASS ----------------

        self.declare_parameter("marker_reached_pause_sec", 0.35)

        # -1.0 = об'їзд справа
        # +1.0 = об'їзд зліва
        self.declare_parameter("bypass_direction", -1.0)

        # 1) Плавна дуга вбік.
        self.declare_parameter("bypass_arc_linear_speed", 0.035)
        self.declare_parameter("bypass_arc_angular_speed", 0.32)
        self.declare_parameter("bypass_arc_duration_sec", 2.0)

        # 2) Проїзд повз маркер.
        self.declare_parameter("bypass_pass_forward_speed", 0.055)
        self.declare_parameter("bypass_pass_forward_duration_sec", 2.0)

        # 3) Легке вирівнювання назад.
        self.declare_parameter("bypass_realign_angular_speed", 0.20)
        self.declare_parameter("bypass_realign_duration_sec", 0.55)

        # Якщо попереду ближче цього значення — спочатку не їдемо вперед,
        # а тільки довертаємо вбік.
        self.declare_parameter("bypass_front_block_m", 0.35)

        # ВАЖЛИВО:
        # Максимум стільки секунд робот може тільки крутитись,
        # якщо попереду близько перешкода.
        # Після цього він переходить до дуги/проїзду, щоб не зависнути.
        self.declare_parameter("bypass_turn_only_when_blocked_sec", 0.9)

        # ---------------- AFTER BYPASS: GO STRAIGHT ----------------

        self.declare_parameter("after_bypass_ignore_sec", 2.5)
        self.declare_parameter("cruise_forward_speed", 0.06)
        self.declare_parameter("cruise_front_block_m", 0.26)
        self.declare_parameter("search_if_cruise_blocked", True)

        # ---------------- LIMITS / LOGGING ----------------

        self.declare_parameter("max_cmd_linear", 0.20)
        self.declare_parameter("max_cmd_angular", 1.20)
        self.declare_parameter("status_log_period_sec", 2.0)

        # =========================================================
        # READ PARAMETERS
        # =========================================================

        self.aruco_status_topic = str(self.get_parameter("aruco_status_topic").value)
        self.front_distance_topic = str(self.get_parameter("front_distance_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.control_hz = float(self.get_parameter("control_hz").value)

        self.aruco_lost_grace_sec = float(
            self.get_parameter("aruco_lost_grace_sec").value
        )

        self.center_deadzone_px = float(self.get_parameter("center_deadzone_px").value)
        self.aruco_turn_kp = float(self.get_parameter("aruco_turn_kp").value)
        self.max_track_angular = float(self.get_parameter("max_track_angular").value)
        self.approach_speed = float(self.get_parameter("approach_speed").value)

        self.allow_forward_while_turning = bool(
            self.get_parameter("allow_forward_while_turning").value
        )

        self.forward_turn_offset_limit_px = float(
            self.get_parameter("forward_turn_offset_limit_px").value
        )

        self.invert_aruco_turn = bool(
            self.get_parameter("invert_aruco_turn").value
        )

        self.assume_reached_if_lost_distance_m = float(
            self.get_parameter("assume_reached_if_lost_distance_m").value
        )

        self.assume_reached_if_lost_width_px = float(
            self.get_parameter("assume_reached_if_lost_width_px").value
        )

        self.recover_turn_to_last_marker = bool(
            self.get_parameter("recover_turn_to_last_marker").value
        )

        self.recover_turn_speed = float(
            self.get_parameter("recover_turn_speed").value
        )

        self.enable_force_bypass_trigger = bool(
            self.get_parameter("enable_force_bypass_trigger").value
        )

        self.force_bypass_if_front_below_m = float(
            self.get_parameter("force_bypass_if_front_below_m").value
        )

        self.force_bypass_if_width_above_px = float(
            self.get_parameter("force_bypass_if_width_above_px").value
        )

        self.force_bypass_if_camera_est_below_m = float(
            self.get_parameter("force_bypass_if_camera_est_below_m").value
        )

        self.force_reached_max_marker_age_sec = float(
            self.get_parameter("force_reached_max_marker_age_sec").value
        )

        self.use_lidar_for_marker_stop = bool(
            self.get_parameter("use_lidar_for_marker_stop").value
        )

        self.target_marker_distance_m = float(
            self.get_parameter("target_marker_distance_m").value
        )

        self.marker_stop_width_px_override = float(
            self.get_parameter("marker_stop_width_px_override").value
        )

        self.marker_real_size_cm = float(
            self.get_parameter("marker_real_size_cm").value
        )

        self.camera_hfov_deg = float(
            self.get_parameter("camera_hfov_deg").value
        )

        self.emergency_reached_distance_m = float(
            self.get_parameter("emergency_reached_distance_m").value
        )

        self.search_turn_speed = float(
            self.get_parameter("search_turn_speed").value
        )

        self.search_turn_duration_sec = float(
            self.get_parameter("search_turn_duration_sec").value
        )

        self.search_pause_sec = float(
            self.get_parameter("search_pause_sec").value
        )

        self.marker_reached_pause_sec = float(
            self.get_parameter("marker_reached_pause_sec").value
        )

        self.bypass_direction = float(self.get_parameter("bypass_direction").value)
        self.bypass_direction = 1.0 if self.bypass_direction >= 0.0 else -1.0

        self.bypass_arc_linear_speed = float(
            self.get_parameter("bypass_arc_linear_speed").value
        )

        self.bypass_arc_angular_speed = float(
            self.get_parameter("bypass_arc_angular_speed").value
        )

        self.bypass_arc_duration_sec = float(
            self.get_parameter("bypass_arc_duration_sec").value
        )

        self.bypass_pass_forward_speed = float(
            self.get_parameter("bypass_pass_forward_speed").value
        )

        self.bypass_pass_forward_duration_sec = float(
            self.get_parameter("bypass_pass_forward_duration_sec").value
        )

        self.bypass_realign_angular_speed = float(
            self.get_parameter("bypass_realign_angular_speed").value
        )

        self.bypass_realign_duration_sec = float(
            self.get_parameter("bypass_realign_duration_sec").value
        )

        self.bypass_front_block_m = float(
            self.get_parameter("bypass_front_block_m").value
        )

        self.bypass_turn_only_when_blocked_sec = float(
            self.get_parameter("bypass_turn_only_when_blocked_sec").value
        )

        self.after_bypass_ignore_sec = float(
            self.get_parameter("after_bypass_ignore_sec").value
        )

        self.cruise_forward_speed = float(
            self.get_parameter("cruise_forward_speed").value
        )

        self.cruise_front_block_m = float(
            self.get_parameter("cruise_front_block_m").value
        )

        self.search_if_cruise_blocked = bool(
            self.get_parameter("search_if_cruise_blocked").value
        )

        self.max_cmd_linear = float(self.get_parameter("max_cmd_linear").value)
        self.max_cmd_angular = float(self.get_parameter("max_cmd_angular").value)

        self.status_log_period_sec = float(
            self.get_parameter("status_log_period_sec").value
        )

        # =========================================================
        # STATE
        # =========================================================

        self.state = "SEARCH_ARUCO"
        self.state_start_time = self.now_sec()

        self.search_turning = True
        self.search_phase_start_time = self.now_sec()

        self.ignore_aruco_until = 0.0

        self.aruco_found = False
        self.aruco_active = False

        self.aruco_id = -1
        self.aruco_offset_px = 0.0
        self.aruco_width_px = 0.0
        self.aruco_height_px = 0.0
        self.aruco_image_width = 0
        self.aruco_image_height = 0
        self.aruco_mode = "UNKNOWN"

        self.last_aruco_time = 0.0

        self.front_distance_m = None
        self.last_front_distance_time = 0.0

        self.marker_counter = 0
        self.estimated_marker_distance_m = None

        self.last_status_log_time = 0.0

        # =========================================================
        # ROS
        # =========================================================

        self.aruco_sub = self.create_subscription(
            String,
            self.aruco_status_topic,
            self.aruco_status_callback,
            10,
        )

        self.front_distance_sub = self.create_subscription(
            Float32,
            self.front_distance_topic,
            self.front_distance_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_topic,
            10,
        )

        timer_period = 1.0 / max(self.control_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            "✅ mission_master started | "
            f"aruco={self.aruco_status_topic}, "
            f"front={self.front_distance_topic}, "
            f"cmd={self.cmd_topic}, "
            f"target_marker_distance={self.target_marker_distance_m:.2f}m"
        )

    # =========================================================
    # UTILS
    # =========================================================

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def set_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f"🔁 STATE: {self.state} -> {new_state}")
            self.state = new_state
            self.state_start_time = self.now_sec()

            if new_state == "SEARCH_ARUCO":
                self.search_turning = True
                self.search_phase_start_time = self.now_sec()

    def reset_marker_memory_after_bypass(self):
        self.aruco_found = False
        self.aruco_active = False
        self.aruco_offset_px = 0.0
        self.aruco_width_px = 0.0
        self.aruco_height_px = 0.0
        self.estimated_marker_distance_m = None

    # =========================================================
    # CALLBACKS
    # =========================================================

    def aruco_status_callback(self, msg):
        now = self.now_sec()

        if now < self.ignore_aruco_until:
            self.aruco_found = False
            self.aruco_active = False
            return

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Bad /aruco/status JSON: {e}")
            self.aruco_found = False
            self.aruco_active = False
            return

        found = bool(data.get("found", False))
        active = bool(data.get("active", found))

        self.aruco_found = found
        self.aruco_active = found and active

        if self.aruco_active:
            self.aruco_id = int(data.get("id", -1))
            self.aruco_offset_px = float(data.get("offset_px", 0.0))
            self.aruco_width_px = float(data.get("width_px", 0.0))
            self.aruco_height_px = float(data.get("height_px", 0.0))

            self.aruco_image_width = int(data.get("image_width", 0))
            self.aruco_image_height = int(data.get("image_height", 0))

            self.aruco_mode = str(data.get("mode", "UNKNOWN"))

            self.last_aruco_time = now
        else:
            self.aruco_active = False

    def front_distance_callback(self, msg):
        value = float(msg.data)

        if math.isfinite(value) and value > 0.0:
            self.front_distance_m = value
            self.last_front_distance_time = self.now_sec()
        else:
            self.front_distance_m = None

    # =========================================================
    # CHECKS
    # =========================================================

    def is_aruco_recent(self):
        now = self.now_sec()

        if now < self.ignore_aruco_until:
            return False

        if self.last_aruco_time <= 0.0:
            return False

        return (now - self.last_aruco_time) <= self.aruco_lost_grace_sec

    def is_force_marker_context_recent(self):
        now = self.now_sec()

        if now < self.ignore_aruco_until:
            return False

        if self.last_aruco_time <= 0.0:
            return False

        return (now - self.last_aruco_time) <= self.force_reached_max_marker_age_sec

    def estimate_distance_from_marker_width(self):
        if self.aruco_width_px <= 1.0:
            return None

        if self.aruco_image_width <= 0:
            return None

        hfov_rad = math.radians(self.camera_hfov_deg)

        if hfov_rad <= 0.0:
            return None

        focal_px = (self.aruco_image_width * 0.5) / math.tan(hfov_rad * 0.5)

        distance_cm = (self.marker_real_size_cm * focal_px) / self.aruco_width_px
        distance_m = distance_cm / 100.0

        if not math.isfinite(distance_m) or distance_m <= 0.0:
            return None

        return distance_m

    def force_marker_reached_check(self):
        if not self.enable_force_bypass_trigger:
            return False, "disabled"

        if not self.is_force_marker_context_recent():
            return False, "marker_context_too_old"

        if self.front_distance_m is not None:
            if self.front_distance_m <= self.force_bypass_if_front_below_m:
                return True, f"force_front {self.front_distance_m:.3f}m"

        if self.aruco_width_px >= self.force_bypass_if_width_above_px:
            return True, f"force_width {self.aruco_width_px:.1f}px"

        est = self.estimate_distance_from_marker_width()
        self.estimated_marker_distance_m = est

        if est is not None and est <= self.force_bypass_if_camera_est_below_m:
            return True, f"force_camera_est {est:.3f}m"

        return False, "not_forced"

    def is_marker_reached(self):
        if self.use_lidar_for_marker_stop and self.front_distance_m is not None:
            if self.front_distance_m <= self.emergency_reached_distance_m:
                return True, f"emergency_lidar {self.front_distance_m:.3f}m"

            if self.front_distance_m <= self.target_marker_distance_m:
                return True, f"lidar_target {self.front_distance_m:.3f}m"

        if self.marker_stop_width_px_override > 0.0:
            if self.aruco_width_px >= self.marker_stop_width_px_override:
                return True, f"width_override {self.aruco_width_px:.1f}px"

        self.estimated_marker_distance_m = self.estimate_distance_from_marker_width()

        if self.estimated_marker_distance_m is not None:
            if self.estimated_marker_distance_m <= self.target_marker_distance_m:
                return True, f"camera_est {self.estimated_marker_distance_m:.3f}m"

        return False, "not_reached"

    # =========================================================
    # COMMANDS
    # =========================================================

    def publish_cmd(self, linear_x, angular_z):
        linear_x = self.clamp(
            float(linear_x),
            -self.max_cmd_linear,
            self.max_cmd_linear,
        )

        angular_z = self.clamp(
            float(angular_z),
            -self.max_cmd_angular,
            self.max_cmd_angular,
        )

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):
        try:
            self.log_status_periodically()

            if self.state in [
                "MARKER_REACHED",
                "BYPASS_ARC_OUT",
                "BYPASS_PASS_FORWARD",
                "BYPASS_REALIGN",
                "CRUISE_FORWARD",
            ]:
                self.handle_bypass_and_cruise()
                return

            if self.state == "SEARCH_ARUCO":
                self.handle_search_aruco()
                return

            if self.state == "TRACK_ARUCO":
                self.handle_track_aruco()
                return

            self.get_logger().warn(f"⚠️ Unknown state: {self.state}")
            self.stop_robot()
            self.set_state("SEARCH_ARUCO")

        except Exception as e:
            self.get_logger().error(f"❌ control_loop error: {e}")
            self.stop_robot()

    # =========================================================
    # SEARCH
    # =========================================================

    def handle_search_aruco(self):
        forced, reason = self.force_marker_reached_check()

        if forced:
            self.marker_counter += 1

            self.get_logger().info(
                f"✅ MARKER FORCE REACHED #{self.marker_counter} from SEARCH | "
                f"reason={reason} | "
                f"width={self.aruco_width_px:.1f}px | "
                f"front={self.front_distance_m}"
            )

            self.stop_robot()
            self.set_state("MARKER_REACHED")
            return

        if self.is_aruco_recent():
            self.stop_robot()
            self.set_state("TRACK_ARUCO")
            return

        now = self.now_sec()
        elapsed = now - self.search_phase_start_time

        if self.search_turning:
            self.publish_cmd(0.0, self.search_turn_speed)

            if elapsed >= self.search_turn_duration_sec:
                self.search_turning = False
                self.search_phase_start_time = now
                self.stop_robot()

            return

        self.stop_robot()

        if elapsed >= self.search_pause_sec:
            self.search_turning = True
            self.search_phase_start_time = now

    # =========================================================
    # TRACK
    # =========================================================

    def handle_track_aruco(self):
        forced, reason = self.force_marker_reached_check()

        if forced:
            self.marker_counter += 1

            self.get_logger().info(
                f"✅ MARKER FORCE REACHED #{self.marker_counter} from TRACK | "
                f"reason={reason} | "
                f"width={self.aruco_width_px:.1f}px | "
                f"front={self.front_distance_m}"
            )

            self.stop_robot()
            self.set_state("MARKER_REACHED")
            return

        if not self.is_aruco_recent():
            self.stop_robot()
            self.set_state("SEARCH_ARUCO")
            return

        if not self.aruco_active:
            if self.front_distance_m is not None:
                if self.front_distance_m <= self.assume_reached_if_lost_distance_m:
                    self.marker_counter += 1

                    self.get_logger().info(
                        f"✅ MARKER ASSUMED REACHED after loss #{self.marker_counter} | "
                        f"front={self.front_distance_m:.3f}m, "
                        f"last_width={self.aruco_width_px:.1f}px"
                    )

                    self.stop_robot()
                    self.set_state("MARKER_REACHED")
                    return

            if self.aruco_width_px >= self.assume_reached_if_lost_width_px:
                self.marker_counter += 1

                self.get_logger().info(
                    f"✅ MARKER ASSUMED REACHED by last width #{self.marker_counter} | "
                    f"last_width={self.aruco_width_px:.1f}px"
                )

                self.stop_robot()
                self.set_state("MARKER_REACHED")
                return

            if self.recover_turn_to_last_marker:
                if abs(self.aruco_offset_px) > self.center_deadzone_px:
                    angular = (
                        -self.recover_turn_speed
                        if self.aruco_offset_px > 0.0
                        else self.recover_turn_speed
                    )

                    if self.invert_aruco_turn:
                        angular = -angular

                    self.publish_cmd(0.0, angular)
                else:
                    self.stop_robot()
            else:
                self.stop_robot()

            return

        reached, reason = self.is_marker_reached()

        if reached:
            self.marker_counter += 1

            self.get_logger().info(
                f"✅ MARKER REACHED #{self.marker_counter} | "
                f"reason={reason} | "
                f"id={self.aruco_id}, "
                f"offset={self.aruco_offset_px:.1f}px, "
                f"width={self.aruco_width_px:.1f}px, "
                f"front={self.front_distance_m}"
            )

            self.stop_robot()
            self.set_state("MARKER_REACHED")
            return

        offset = float(self.aruco_offset_px)

        if abs(offset) > self.center_deadzone_px:
            angular = -self.aruco_turn_kp * offset

            if self.invert_aruco_turn:
                angular = -angular

            angular = self.clamp(
                angular,
                -self.max_track_angular,
                self.max_track_angular,
            )

            if self.allow_forward_while_turning:
                if abs(offset) <= self.forward_turn_offset_limit_px:
                    linear = self.approach_speed
                else:
                    linear = 0.0
            else:
                linear = 0.0

            self.publish_cmd(linear, angular)
            return

        self.publish_cmd(self.approach_speed, 0.0)

    # =========================================================
    # CURVE BYPASS + CRUISE
    # =========================================================

    def handle_bypass_and_cruise(self):
        now = self.now_sec()
        elapsed = now - self.state_start_time

        # =====================================================
        # 1. Пауза після досягнення маркера
        # =====================================================
        if self.state == "MARKER_REACHED":
            self.stop_robot()

            if elapsed >= self.marker_reached_pause_sec:
                self.set_state("BYPASS_ARC_OUT")

            return

        # =====================================================
        # 2. Плавна дуга вбік навколо маркера
        # =====================================================
        if self.state == "BYPASS_ARC_OUT":
            angular = self.bypass_direction * self.bypass_arc_angular_speed

            front_blocked = (
                self.front_distance_m is not None
                and self.front_distance_m < self.bypass_front_block_m
            )

            # Якщо попереду близько — спочатку тільки довертаємо вбік,
            # але НЕ нескінченно.
            if front_blocked and elapsed < self.bypass_turn_only_when_blocked_sec:
                self.publish_cmd(0.0, angular)

                self.get_logger().warn(
                    f"🟡 BYPASS_ARC_OUT blocked | "
                    f"front={self.front_distance_m:.3f}m < {self.bypass_front_block_m:.3f}m | "
                    f"turning only, elapsed={elapsed:.2f}s"
                )

                return

            # Після короткого довернення рухаємося дугою.
            self.publish_cmd(self.bypass_arc_linear_speed, angular)

            if elapsed >= self.bypass_arc_duration_sec:
                self.stop_robot()
                self.set_state("BYPASS_PASS_FORWARD")

            return

        # =====================================================
        # 3. Проїхати повз мітку
        # =====================================================
        if self.state == "BYPASS_PASS_FORWARD":
            front_blocked = (
                self.front_distance_m is not None
                and self.front_distance_m < self.bypass_front_block_m
            )

            # Якщо попереду близько — коротко довертаємо вбік,
            # але не зависаємо тут назавжди.
            if front_blocked and elapsed < self.bypass_turn_only_when_blocked_sec:
                angular = self.bypass_direction * self.bypass_arc_angular_speed
                self.publish_cmd(0.0, angular)

                self.get_logger().warn(
                    f"🟡 BYPASS_PASS_FORWARD blocked | "
                    f"front={self.front_distance_m:.3f}m < {self.bypass_front_block_m:.3f}m | "
                    f"turning only, elapsed={elapsed:.2f}s"
                )

                return

            self.publish_cmd(self.bypass_pass_forward_speed, 0.0)

            if elapsed >= self.bypass_pass_forward_duration_sec:
                self.stop_robot()
                self.set_state("BYPASS_REALIGN")

            return

        # =====================================================
        # 4. Легке вирівнювання назад
        # =====================================================
        if self.state == "BYPASS_REALIGN":
            angular = -self.bypass_direction * self.bypass_realign_angular_speed
            self.publish_cmd(0.0, angular)

            if elapsed >= self.bypass_realign_duration_sec:
                self.stop_robot()

                self.ignore_aruco_until = now + self.after_bypass_ignore_sec
                self.reset_marker_memory_after_bypass()

                self.get_logger().info(
                    f"✅ CURVE BYPASS DONE | now cruising forward, "
                    f"ignoring old marker for {self.after_bypass_ignore_sec:.1f}s"
                )

                self.set_state("CRUISE_FORWARD")

            return

        # =====================================================
        # 5. Після об'їзду — продовжити рух прямо
        # =====================================================
        if self.state == "CRUISE_FORWARD":
            if self.is_aruco_recent():
                self.stop_robot()
                self.set_state("TRACK_ARUCO")
                return

            if self.front_distance_m is not None and self.front_distance_m < self.cruise_front_block_m:
                self.stop_robot()

                if self.search_if_cruise_blocked:
                    self.get_logger().warn(
                        f"⚠️ Cruise blocked by front obstacle: "
                        f"{self.front_distance_m:.3f}m -> SEARCH_ARUCO"
                    )
                    self.set_state("SEARCH_ARUCO")

                return

            self.publish_cmd(self.cruise_forward_speed, 0.0)
            return

    # =========================================================
    # LOGGING
    # =========================================================

    def log_status_periodically(self):
        now = self.now_sec()

        if now - self.last_status_log_time < self.status_log_period_sec:
            return

        self.last_status_log_time = now

        if self.last_aruco_time > 0.0:
            aruco_age_txt = f"{now - self.last_aruco_time:.2f}s"
        else:
            aruco_age_txt = "never"

        if self.front_distance_m is not None:
            front_txt = f"{self.front_distance_m:.3f}m"
        else:
            front_txt = "None"

        if self.estimated_marker_distance_m is not None:
            est_txt = f"{self.estimated_marker_distance_m:.3f}m"
        else:
            est_txt = "None"

        self.get_logger().info(
            f"📊 MISSION | state={self.state} | "
            f"aruco_active={self.aruco_active} "
            f"id={self.aruco_id} "
            f"offset={self.aruco_offset_px:.1f}px "
            f"width={self.aruco_width_px:.1f}px "
            f"age={aruco_age_txt} | "
            f"front={front_txt} | "
            f"cam_est={est_txt} | "
            f"markers_passed={self.marker_counter}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = MissionMaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()