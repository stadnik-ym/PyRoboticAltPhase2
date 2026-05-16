#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs import msg
from std_msgs.msg import Float32, Bool


class LidarSafety(Node):
    """
    Manual driving lidar safety node.

    Input:
      /cmd_vel_raw              from keycontrol.py
      /lidar/front_distance     from ld06_node.py

    Output:
      /cmd_vel                  to motor_node

    Logic:
      if moving forward and front distance <= stop_distance_m:
          block forward movement
      rotation and reverse are allowed by default
    """

    def __init__(self):
        super().__init__('lidar_safety')

       

        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('front_distance_topic', '/lidar/front_distance')
        self.declare_parameter('safety_stop_topic', '/safety/lidar_stop')

        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('lidar_timeout_sec', 0.7)

        # На якій дистанції зупиняти рух вперед.
        self.declare_parameter('stop_distance_m', 0.22)

        # На якій дистанції знову дозволити рух вперед.
        # Має бути трохи більше stop_distance_m, щоб робот не смикався.
        self.declare_parameter('clear_distance_m', 0.30)

        self.declare_parameter('invalid_front_timeout_sec', 1.0)

        # Якщо True — коли спереду перешкода, можна крутитися на місці.
        self.declare_parameter('allow_rotation_when_blocked', True)

        # Якщо True — коли спереду перешкода, можна їхати назад.
        self.declare_parameter('allow_reverse_when_blocked', True)

        # Обмеження швидкості.
        self.declare_parameter('max_linear_x', 0.35)
        self.declare_parameter('max_angular_z', 1.5)

        # Якщо True — перед стопом робот плавно сповільнюється.
        self.declare_parameter('enable_slowdown', True)
        self.declare_parameter('slowdown_distance_m', 0.80)
        self.declare_parameter('min_slowdown_factor', 0.25)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.front_distance_topic = self.get_parameter('front_distance_topic').value
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').value

        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.lidar_timeout_sec = float(self.get_parameter('lidar_timeout_sec').value)

        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.clear_distance_m = float(self.get_parameter('clear_distance_m').value)

        self.invalid_front_timeout_sec = float(
            self.get_parameter('invalid_front_timeout_sec').value
        )

        self.last_valid_front_time = 0.0

        self.allow_rotation_when_blocked = bool(
            self.get_parameter('allow_rotation_when_blocked').value
        )
        self.allow_reverse_when_blocked = bool(
            self.get_parameter('allow_reverse_when_blocked').value
        )

        self.max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))

        self.enable_slowdown = bool(self.get_parameter('enable_slowdown').value)
        self.slowdown_distance_m = float(self.get_parameter('slowdown_distance_m').value)
        self.min_slowdown_factor = float(self.get_parameter('min_slowdown_factor').value)

        self.min_slowdown_factor = max(0.0, min(1.0, self.min_slowdown_factor))

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0

        self.front_distance = -1.0
        self.last_lidar_time = 0.0

        self.front_blocked = False

        self.cmd_sub = self.create_subscription(
            Twist,
            self.input_cmd_topic,
            self.cmd_callback,
            1
        )

        self.front_sub = self.create_subscription(
            Float32,
            self.front_distance_topic,
            self.front_distance_callback,
            1
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_cmd_topic,
            1
        )

        self.stop_pub = self.create_publisher(
            Bool,
            self.safety_stop_topic,
            10
        )

        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info(
            f'Lidar safety started: {self.input_cmd_topic} -> {self.output_cmd_topic}, '
            f'front_distance_topic={self.front_distance_topic}, '
            f'stop={self.stop_distance_m}, clear={self.clear_distance_m}'
        )

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def front_distance_callback(self, msg: Float32):
        now = time.time()
        distance = float(msg.data)

        self.last_lidar_time = now

        if distance >= 0.0:
            self.front_distance = distance
            self.last_valid_front_time = now

            if distance <= self.stop_distance_m:
                self.front_blocked = True

                if self.last_cmd.linear.x > 0.0:
                    stop_cmd = self.make_stop_cmd()

                    if self.allow_rotation_when_blocked:
                        stop_cmd.angular.z = self.last_cmd.angular.z

                    self.cmd_pub.publish(stop_cmd)
                    self.publish_stop_state(True)

    def make_stop_cmd(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg

    def clamp_cmd(self, cmd: Twist):
        if cmd.linear.x > self.max_linear_x:
            cmd.linear.x = self.max_linear_x
        elif cmd.linear.x < -self.max_linear_x:
            cmd.linear.x = -self.max_linear_x

        if cmd.angular.z > self.max_angular_z:
            cmd.angular.z = self.max_angular_z
        elif cmd.angular.z < -self.max_angular_z:
            cmd.angular.z = -self.max_angular_z

        # Диференційний робот не має їхати боком.
        cmd.linear.y = 0.0

        return cmd

    def valid_front_distance(self):
        now = time.time()
        return (
            self.front_distance >= 0.0
            and now - self.last_valid_front_time <= self.invalid_front_timeout_sec
    )

    def update_blocked_state(self):
        """
        Hysteresis:
        not blocked -> blocked when distance <= stop_distance_m
        blocked -> clear only when valid distance >= clear_distance_m

        Important:
        invalid / -1.0 must NOT clear blocked state.
        """

        if not self.valid_front_distance():
        # Якщо дані тимчасово пропали:
        # - не очищаємо blocked
        # - якщо вже були заблоковані, залишаємо blocked=True
            return

        if self.front_blocked:
            if self.front_distance >= self.clear_distance_m:
                self.front_blocked = False
        else:
            if self.front_distance <= self.stop_distance_m:
                self.front_blocked = True

    def slowdown_factor(self):
        if not self.enable_slowdown:
            return 1.0

        if not self.valid_front_distance():
            return 1.0

        if self.front_distance >= self.slowdown_distance_m:
            return 1.0

        if self.slowdown_distance_m <= 0.01:
            return 1.0

        factor = self.front_distance / self.slowdown_distance_m
        factor = max(self.min_slowdown_factor, min(1.0, factor))
        return factor

    def publish_stop_state(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.stop_pub.publish(msg)

    def control_loop(self):
        now = time.time()

        if now - self.last_cmd_time > self.cmd_timeout_sec:
            self.cmd_pub.publish(self.make_stop_cmd())
            self.publish_stop_state(True)
            return

        if now - self.last_lidar_time > self.lidar_timeout_sec:
            self.get_logger().warn('LIDAR TIMEOUT: stopping robot')
            self.cmd_pub.publish(self.make_stop_cmd())
            self.publish_stop_state(True)
            return

        self.update_blocked_state()

        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x
        cmd.linear.y = self.last_cmd.linear.y
        cmd.linear.z = self.last_cmd.linear.z
        cmd.angular.x = self.last_cmd.angular.x
        cmd.angular.y = self.last_cmd.angular.y
        cmd.angular.z = self.last_cmd.angular.z

        cmd = self.clamp_cmd(cmd)

        moving_forward = cmd.linear.x > 0.0
        moving_backward = cmd.linear.x < 0.0

        safety_active = True
        reason = 'OK'

        if self.front_blocked and moving_forward:
            cmd.linear.x = 0.0
            safety_active = True
            reason = 'front blocked'

            if not self.allow_rotation_when_blocked:
                cmd.angular.z = 0.0

        if self.front_blocked and moving_backward and not self.allow_reverse_when_blocked:
            cmd.linear.x = 0.0
            safety_active = True
            reason = 'reverse disabled while blocked'

        if not self.front_blocked and moving_forward:
            factor = self.slowdown_factor()
            if factor < 1.0:
                cmd.linear.x *= factor
                safety_active = True
                reason = f'slowdown {factor:.2f}'

        cmd = self.clamp_cmd(cmd)

        self.cmd_pub.publish(cmd)
        self.publish_stop_state(safety_active)

        self.log_status(cmd, safety_active, reason)

    def log_status(self, cmd, safety_active, reason):
        now = time.time()

        if not hasattr(self, '_last_log'):
            self._last_log = 0.0

        if now - self._last_log < 0.5:
            return

        self._last_log = now

        if safety_active:
            self.get_logger().warn(
                f'SAFETY: {reason} | '
                f'front={self.front_distance:.3f} m | '
                f'out linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}'
            )
        else:
            self.get_logger().info(
                f'OK | '
                f'front={self.front_distance:.3f} m | '
                f'out linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = LidarSafety()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()