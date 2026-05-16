# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry

# import lgpio
# import time
# import math


# ENA = 17
# IN1 = 27
# IN2 = 22

# ENB = 13
# IN3 = 26
# IN4 = 19

# PWM_FREQ = 700

# # Мінімальна потужність PWM у %
# # Якщо мотори пищать, але не рушають — підніми до 55 або 65
# MIN_PWM = 65

# WHEEL_BASE = 0.16
# MAX_LINEAR = 0.2
# MAX_ANGULAR = 2.0

# CMD_TIMEOUT = 0.5


# class DiffDrive(Node):

#     def __init__(self):
#         super().__init__('diff_drive_node')

#         self.get_logger().info("Starting DiffDrive")

#         self.sub = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_cb,
#             1
#         )

#         self.odom_pub = self.create_publisher(
#             Odometry,
#             '/odom',
#             10
#         )

#         self.timer = self.create_timer(0.01, self.update)

#         self.chip = lgpio.gpiochip_open(4)

#         self.setup_gpio()

#         self.v = 0.0
#         self.w = 0.0
#         self.last_cmd = time.time()

#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0

#         self.last_time = time.time()

#         self.get_logger().info("DiffDrive Ready. Listening /cmd_vel")

#     def setup_gpio(self):
#         pins = [ENA, IN1, IN2, ENB, IN3, IN4]

#         for p in pins:
#             lgpio.gpio_claim_output(self.chip, p)

#         self.stop()

#     def cmd_cb(self, msg: Twist):
#         self.v = max(-MAX_LINEAR, min(MAX_LINEAR, msg.linear.x))
#         self.w = max(-MAX_ANGULAR, min(MAX_ANGULAR, msg.angular.z))
#         self.last_cmd = time.time()

#         # Миттєва реакція на стоп-команду від lidar_safety
#         if abs(self.v) < 0.001 and abs(self.w) < 0.001:
#             self.stop()

#     def pwm_from_speed(self, s):
#         duty = int(abs(s) * 100)

#         if duty > 0:
#             duty = max(MIN_PWM, duty)

#         duty = max(0, min(100, duty))
#         return duty

#     def set_left(self, s):
#         duty = self.pwm_from_speed(s)

#         if s > 0:
#             lgpio.gpio_write(self.chip, IN1, 1)
#             lgpio.gpio_write(self.chip, IN2, 0)
#         elif s < 0:
#             lgpio.gpio_write(self.chip, IN1, 0)
#             lgpio.gpio_write(self.chip, IN2, 1)
#         else:
#             lgpio.gpio_write(self.chip, IN1, 0)
#             lgpio.gpio_write(self.chip, IN2, 0)

#         lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)

#     def set_right(self, s):
#         duty = self.pwm_from_speed(s)

#         if s > 0:
#             lgpio.gpio_write(self.chip, IN3, 1)
#             lgpio.gpio_write(self.chip, IN4, 0)
#         elif s < 0:
#             lgpio.gpio_write(self.chip, IN3, 0)
#             lgpio.gpio_write(self.chip, IN4, 1)
#         else:
#             lgpio.gpio_write(self.chip, IN3, 0)
#             lgpio.gpio_write(self.chip, IN4, 0)

#         lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)

#     def stop(self):
#         lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
#         lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

#         for p in [IN1, IN2, IN3, IN4]:
#             lgpio.gpio_write(self.chip, p, 0)

#     def update(self):
#         now = time.time()

#         dt = now - self.last_time
#         self.last_time = now

#         if now - self.last_cmd > CMD_TIMEOUT:
#             self.v = 0.0
#             self.w = 0.0

#         vl = self.v - self.w * WHEEL_BASE / 2
#         vr = self.v + self.w * WHEEL_BASE / 2

#         maxv = max(abs(vl), abs(vr))

#         if maxv > MAX_LINEAR:
#             k = MAX_LINEAR / maxv
#             vl *= k
#             vr *= k

#         left = vl / MAX_LINEAR
#         right = vr / MAX_LINEAR

#         self.set_left(left)
#         self.set_right(right)

#         v = (vl + vr) / 2
#         w = (vr - vl) / WHEEL_BASE

#         self.theta += w * dt

#         self.x += v * math.cos(self.theta) * dt
#         self.y += v * math.sin(self.theta) * dt

#         self.publish_odom(v, w)

#     def publish_odom(self, v, w):
#         msg = Odometry()

#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "odom"
#         msg.child_frame_id = "base_link"

#         msg.pose.pose.position.x = self.x
#         msg.pose.pose.position.y = self.y

#         msg.pose.pose.orientation.z = math.sin(self.theta / 2)
#         msg.pose.pose.orientation.w = math.cos(self.theta / 2)

#         msg.twist.twist.linear.x = v
#         msg.twist.twist.angular.z = w

#         self.odom_pub.publish(msg)

#     def destroy_node(self):
#         try:
#             self.stop()
#             lgpio.gpiochip_close(self.chip)
#         except Exception:
#             pass

#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)

#     node = DiffDrive()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import lgpio
import time
import math


ENA = 17
IN1 = 27
IN2 = 22

ENB = 13
IN3 = 26
IN4 = 19

PWM_FREQ = 700

# ============================================================
# НАЛАШТУВАННЯ МОТОРІВ
# ============================================================

# Мінімальний PWM окремо для кожного мотора
LEFT_MIN_PWM = 90
RIGHT_MIN_PWM = 90

# Компенсація моторів
# Якщо робот тягне вправо — збільш RIGHT_GAIN або зменш LEFT_GAIN
# Якщо робот тягне вліво — збільш LEFT_GAIN або зменш RIGHT_GAIN
LEFT_GAIN = 1.00
RIGHT_GAIN = 1.00

# Додаткова компенсація саме для руху вперед/назад
LEFT_FORWARD_GAIN = 1.00
RIGHT_FORWARD_GAIN = 1.00

LEFT_BACKWARD_GAIN = 1.00
RIGHT_BACKWARD_GAIN = 1.00

# Якщо один мотор фізично стоїть навпаки, можна інвертувати
LEFT_INVERT = False
RIGHT_INVERT = False


WHEEL_BASE = 0.16
MAX_LINEAR = 0.2
MAX_ANGULAR = 2.0

CMD_TIMEOUT = 0.5


class DiffDrive(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        self.get_logger().info("Starting DiffDrive with motor compensation")

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            1
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.chip = lgpio.gpiochip_open(4)

        self.setup_gpio()

        self.v = 0.0
        self.w = 0.0
        self.last_cmd = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()

        self.timer = self.create_timer(0.01, self.update)

        self.get_logger().info("DiffDrive Ready. Listening /cmd_vel")
        self.get_logger().info(
            f"Motor compensation: "
            f"LEFT_GAIN={LEFT_GAIN}, RIGHT_GAIN={RIGHT_GAIN}, "
            f"LEFT_MIN_PWM={LEFT_MIN_PWM}, RIGHT_MIN_PWM={RIGHT_MIN_PWM}"
        )

    def setup_gpio(self):
        pins = [ENA, IN1, IN2, ENB, IN3, IN4]

        for p in pins:
            lgpio.gpio_claim_output(self.chip, p)

        self.stop()

    def cmd_cb(self, msg: Twist):
        self.v = max(-MAX_LINEAR, min(MAX_LINEAR, msg.linear.x))
        self.w = max(-MAX_ANGULAR, min(MAX_ANGULAR, msg.angular.z))
        self.last_cmd = time.time()

        if abs(self.v) < 0.001 and abs(self.w) < 0.001:
            self.stop()

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def pwm_from_speed(self, s, min_pwm):
        """
        s очікується в діапазоні від -1.0 до 1.0.
        PWM рахується не як max(MIN_PWM, speed*100),
        а як плавне масштабування від min_pwm до 100.
        """

        speed = abs(s)

        if speed < 0.001:
            return 0

        speed = self.clamp(speed, 0.0, 1.0)

        duty = min_pwm + speed * (100.0 - min_pwm)
        duty = int(self.clamp(duty, 0, 100))

        return duty

    def apply_left_compensation(self, s):
        """
        Компенсація лівого мотора.
        """

        if LEFT_INVERT:
            s = -s

        s *= LEFT_GAIN

        if s > 0:
            s *= LEFT_FORWARD_GAIN
        elif s < 0:
            s *= LEFT_BACKWARD_GAIN

        return self.clamp(s, -1.0, 1.0)

    def apply_right_compensation(self, s):
        """
        Компенсація правого мотора.
        """

        if RIGHT_INVERT:
            s = -s

        s *= RIGHT_GAIN

        if s > 0:
            s *= RIGHT_FORWARD_GAIN
        elif s < 0:
            s *= RIGHT_BACKWARD_GAIN

        return self.clamp(s, -1.0, 1.0)

    def set_left(self, s):
        s = self.apply_left_compensation(s)
        duty = self.pwm_from_speed(s, LEFT_MIN_PWM)

        if s > 0:
            lgpio.gpio_write(self.chip, IN1, 1)
            lgpio.gpio_write(self.chip, IN2, 0)
        elif s < 0:
            lgpio.gpio_write(self.chip, IN1, 0)
            lgpio.gpio_write(self.chip, IN2, 1)
        else:
            lgpio.gpio_write(self.chip, IN1, 0)
            lgpio.gpio_write(self.chip, IN2, 0)

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)

    def set_right(self, s):
        s = self.apply_right_compensation(s)
        duty = self.pwm_from_speed(s, RIGHT_MIN_PWM)

        if s > 0:
            lgpio.gpio_write(self.chip, IN3, 1)
            lgpio.gpio_write(self.chip, IN4, 0)
        elif s < 0:
            lgpio.gpio_write(self.chip, IN3, 0)
            lgpio.gpio_write(self.chip, IN4, 1)
        else:
            lgpio.gpio_write(self.chip, IN3, 0)
            lgpio.gpio_write(self.chip, IN4, 0)

        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)

    def stop(self):
        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

        for p in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_write(self.chip, p, 0)

    def update(self):
        now = time.time()

        dt = now - self.last_time
        self.last_time = now

        if now - self.last_cmd > CMD_TIMEOUT:
            self.v = 0.0
            self.w = 0.0

        vl = self.v - self.w * WHEEL_BASE / 2.0
        vr = self.v + self.w * WHEEL_BASE / 2.0

        maxv = max(abs(vl), abs(vr))

        if maxv > MAX_LINEAR:
            k = MAX_LINEAR / maxv
            vl *= k
            vr *= k

        left = vl / MAX_LINEAR
        right = vr / MAX_LINEAR

        self.set_left(left)
        self.set_right(right)

        v = (vl + vr) / 2.0
        w = (vr - vl) / WHEEL_BASE

        self.theta += w * dt

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        self.publish_odom(v, w)

    def publish_odom(self, v, w):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.odom_pub.publish(msg)

    def destroy_node(self):
        try:
            self.stop()
            lgpio.gpiochip_close(self.chip)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DiffDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()