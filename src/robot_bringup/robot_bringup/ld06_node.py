#!/usr/bin/env python3

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray

try:
    import serial
except ImportError:
    serial = None


class LD06Node(Node):
    """
    LD06 lidar node.

    Publishes:
      /scan                    sensor_msgs/LaserScan
      /lidar/front_distance    std_msgs/Float32
      /lidar/closest           std_msgs/Float32MultiArray

    /lidar/front_distance:
      distance in meters in front sector.
      -1.0 means no valid point in front sector.

    /lidar/closest:
      [distance_m, angle_deg]
    """

    CRC_TABLE = [
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
        0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
        0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
        0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
        0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
        0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
        0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
        0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
        0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
        0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
        0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
        0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
        0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
        0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
        0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
        0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
        0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
        0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
        0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
        0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
        0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
        0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
    ]

    def __init__(self):
        super().__init__('ld06_node')

        if serial is None:
            raise RuntimeError('pyserial not installed. Use: pip3 install pyserial')
        
        self.declare_parameter('immediate_front_publish', True)
        self.declare_parameter('immediate_front_pub_min_period', 0.005)

        self.immediate_front_publish = bool(
            self.get_parameter('immediate_front_publish').value
        )

        self.immediate_front_pub_min_period = float(
            self.get_parameter('immediate_front_pub_min_period').value
        )


        self.last_immediate_front_pub_time = 0.0

        self.declare_parameter('front_hold_sec', 0.8)
        self.declare_parameter('front_filter_window', 5)

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_distance_topic', '/lidar/front_distance')
        self.declare_parameter('closest_topic', '/lidar/closest')

        self.declare_parameter('publish_rate_hz', 50.0)

        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('range_max_m', 12.0)

        # Для тестів з коробкою краще 0 або 5.
        self.declare_parameter('min_confidence', 0)

        # 1 градус = 360 bins.
        self.declare_parameter('angle_resolution_deg', 1.0)

        # Калібрування напрямку.
        # Після калібрування перешкода спереду має давати angle ~= 0.
        self.declare_parameter('angle_offset_deg', 0.0)

        # Якщо ліво/право дзеркально переплутані.
        self.declare_parameter('invert_angle_direction', False)

        # Передній сектор.
        # Для тесту з коробкою 5 см на 0.55 м краще зробити ширше.
        self.declare_parameter('front_min_deg', -35.0)
        self.declare_parameter('front_max_deg', 35.0)

        # Скільки секунд точка вважається актуальною.
        # LD06 зазвичай проходить коло швидше, але 0.35 дає стабільність.
        self.declare_parameter('point_max_age_sec', 0.35)

        # Мінімальна кількість точок у передньому секторі.
        # 1 = ловить навіть малу коробку/стрічку.
        # 2 або 3 = менше шуму, але може не зловити тонкий об'єкт.
        self.declare_parameter('front_min_points', 1)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.scan_topic = self.get_parameter('scan_topic').value
        self.front_distance_topic = self.get_parameter('front_distance_topic').value
        self.closest_topic = self.get_parameter('closest_topic').value

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.range_min_m = float(self.get_parameter('range_min_m').value)
        self.range_max_m = float(self.get_parameter('range_max_m').value)
        self.min_confidence = int(self.get_parameter('min_confidence').value)

        self.angle_resolution_deg = float(self.get_parameter('angle_resolution_deg').value)
        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)
        self.invert_angle_direction = bool(self.get_parameter('invert_angle_direction').value)

        self.front_min_deg = float(self.get_parameter('front_min_deg').value)
        self.front_max_deg = float(self.get_parameter('front_max_deg').value)
        self.point_max_age_sec = float(self.get_parameter('point_max_age_sec').value)
        self.front_min_points = int(self.get_parameter('front_min_points').value)

        self.front_hold_sec = float(self.get_parameter('front_hold_sec').value)
        self.front_filter_window = int(self.get_parameter('front_filter_window').value)

        self.last_valid_front_distance = -1.0
        self.last_valid_front_time = 0.0
        self.front_history = []

        self.bin_count = int(round(360.0 / self.angle_resolution_deg))

        self.ranges = [math.inf] * self.bin_count
        self.intensities = [0.0] * self.bin_count
        self.update_time = [0.0] * self.bin_count

        self.lock = threading.Lock()
        self.running = True

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.front_pub = self.create_publisher(Float32, self.front_distance_topic, 10)
        self.closest_pub = self.create_publisher(Float32MultiArray, self.closest_topic, 10)

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.005
        )

        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self.publish_data
        )

        self.get_logger().info(
            f'LD06 started on {self.port}, baud={self.baudrate}, '
            f'offset={self.angle_offset_deg}, invert={self.invert_angle_direction}, '
            f'front=[{self.front_min_deg}, {self.front_max_deg}]'
        )

    def crc8(self, data: bytes) -> int:
        crc = 0
        for b in data:
            crc = self.CRC_TABLE[(crc ^ b) & 0xFF]
        return crc

    @staticmethod
    def normalize_0_360(angle_deg):
        angle_deg = angle_deg % 360.0
        if angle_deg < 0.0:
            angle_deg += 360.0
        return angle_deg

    @staticmethod
    def normalize_180(angle_deg):
        return (angle_deg + 180.0) % 360.0 - 180.0

    def corrected_angle(self, raw_angle_deg):
        if self.invert_angle_direction:
            angle = -raw_angle_deg
        else:
            angle = raw_angle_deg

        angle += self.angle_offset_deg
        return self.normalize_0_360(angle)

    def angle_to_index(self, angle_deg):
        return int(round(self.normalize_0_360(angle_deg) / self.angle_resolution_deg)) % self.bin_count

    def angle_in_front(self, angle_deg):
        angle_deg = self.normalize_180(angle_deg)

        if self.front_min_deg <= self.front_max_deg:
            return self.front_min_deg <= angle_deg <= self.front_max_deg

        return angle_deg >= self.front_min_deg or angle_deg <= self.front_max_deg

    def valid_packet(self, packet):
        if len(packet) != 47:
            return False
        if packet[0] != 0x54 or packet[1] != 0x2C:
            return False
        return self.crc8(packet[:46]) == packet[46]

    def read_loop(self):
        buffer = bytearray()

        while self.running and rclpy.ok():
            try:
                waiting = self.serial.in_waiting

                if waiting > 0:
                    data = self.serial.read(waiting)
                else:
                    data = self.serial.read(1)

                if not data:
                    time.sleep(0.001)
                    continue

                buffer.extend(data)

                while len(buffer) >= 47:
                    if buffer[0] != 0x54 or buffer[1] != 0x2C:
                        buffer.pop(0)
                        continue

                    packet = bytes(buffer[:47])
                    del buffer[:47]

                    if self.valid_packet(packet):
                        self.parse_packet(packet)

            except Exception as e:
                self.get_logger().warn(f'LD06 read error: {e}')
                time.sleep(0.05)
    
    def publish_front_immediate(self, distance_m):
        now = time.time()

        if now - self.last_immediate_front_pub_time < self.immediate_front_pub_min_period:
            return

        self.last_immediate_front_pub_time = now

        self.last_valid_front_distance = float(distance_m)
        self.last_valid_front_time = now

        msg = Float32()
        msg.data = float(distance_m)
        self.front_pub.publish(msg)


    def parse_packet(self, packet):
        start_angle = struct.unpack_from('<H', packet, 4)[0] / 100.0
        end_angle = struct.unpack_from('<H', packet, 42)[0] / 100.0

        angle_span = end_angle - start_angle

        if angle_span < -180.0:
            angle_span += 360.0
        elif angle_span > 180.0:
            angle_span -= 360.0

        point_count = 12
        angle_step = angle_span / float(point_count - 1)
        now = time.time()

        front_packet_min = math.inf
        front_packet_points = 0

        with self.lock:
            for i in range(point_count):
                offset = 6 + i * 3

                distance_mm = struct.unpack_from('<H', packet, offset)[0]
                confidence = packet[offset + 2]

                if confidence < self.min_confidence:
                    continue

                if distance_mm <= 0:
                    continue

                distance_m = distance_mm / 1000.0

                if distance_m < self.range_min_m or distance_m > self.range_max_m:
                    continue

                raw_angle = start_angle + angle_step * i
                robot_angle = self.corrected_angle(raw_angle)
                idx = self.angle_to_index(robot_angle)

                old_age = (
                    now - self.update_time[idx]
                    if self.update_time[idx] > 0.0
                    else 999.0
                )

                if old_age > 0.05:
                    self.ranges[idx] = distance_m
                    self.intensities[idx] = float(confidence)
                    self.update_time[idx] = now
                else:
                    if distance_m < self.ranges[idx]:
                        self.ranges[idx] = distance_m
                        self.intensities[idx] = float(confidence)
                        self.update_time[idx] = now

                angle_robot_180 = self.normalize_180(robot_angle)

                if self.angle_in_front(angle_robot_180):
                    front_packet_points += 1

                    if distance_m < front_packet_min:
                        front_packet_min = distance_m

        if (
            self.immediate_front_publish
            and front_packet_points >= self.front_min_points
            and not math.isinf(front_packet_min)
        ):
            self.publish_front_immediate(front_packet_min)

    def get_snapshot(self):
        now = time.time()

        with self.lock:
            ranges = list(self.ranges)
            intensities = list(self.intensities)
            times = list(self.update_time)

        for i in range(self.bin_count):
            if times[i] <= 0.0:
                ranges[i] = math.inf
                intensities[i] = 0.0
            elif now - times[i] > self.point_max_age_sec:
                ranges[i] = math.inf
                intensities[i] = 0.0

        return ranges, intensities

    def compute_front_distance(self, ranges):
        values = []

        for idx, dist in enumerate(ranges):
            if math.isinf(dist) or math.isnan(dist):
                continue

            angle_0_360 = idx * self.angle_resolution_deg
            angle_robot = self.normalize_180(angle_0_360)

            if self.angle_in_front(angle_robot):
                values.append(dist)

        now = time.time()

        if len(values) >= self.front_min_points:
            values.sort()

            # Беремо найближчу валідну точку в передній зоні.
            instant_front = values[0]

            self.front_history.append(instant_front)

            if len(self.front_history) > self.front_filter_window:
                self.front_history.pop(0)

            # Для safety краще брати мінімум з короткої історії,
            # щоб не пропустити перешкоду між обертами лідара.
            filtered_front = min(self.front_history)

            self.last_valid_front_distance = filtered_front
            self.last_valid_front_time = now

            return filtered_front, len(values)

        # Якщо прямо зараз точок нема, але нещодавно були —
        # не публікуємо -1.0 одразу.
        if self.last_valid_front_time > 0.0:
            age = now - self.last_valid_front_time

            if age <= self.front_hold_sec:
                return self.last_valid_front_distance, 0

            return -1.0, 0

        return -1.0, 0

    def compute_closest(self, ranges):
        best_dist = math.inf
        best_angle = None

        for idx, dist in enumerate(ranges):
            if math.isinf(dist) or math.isnan(dist):
                continue

            if dist < best_dist:
                best_dist = dist
                angle_0_360 = idx * self.angle_resolution_deg
                best_angle = self.normalize_180(angle_0_360)

        if best_angle is None:
            return -1.0, 999.0

        return best_dist, best_angle

    def publish_data(self):
        ranges, intensities = self.get_snapshot()

        stamp = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.frame_id

        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = math.radians(self.angle_resolution_deg)

        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(self.publish_rate_hz, 1.0)

        scan.range_min = self.range_min_m
        scan.range_max = self.range_max_m

        scan.ranges = ranges
        scan.intensities = intensities

        self.scan_pub.publish(scan)

        front_distance, front_points = self.compute_front_distance(ranges)
        closest_distance, closest_angle = self.compute_closest(ranges)

        front_msg = Float32()
        front_msg.data = float(front_distance)
        self.front_pub.publish(front_msg)

        closest_msg = Float32MultiArray()
        closest_msg.data = [float(closest_distance), float(closest_angle)]
        self.closest_pub.publish(closest_msg)

        self.log_debug(front_distance, front_points, closest_distance, closest_angle)

    def log_debug(self, front_distance, front_points, closest_distance, closest_angle):
        now = time.time()

        if not hasattr(self, '_last_log'):
            self._last_log = 0.0

        if now - self._last_log < 0.5:
            return

        self._last_log = now

        ###self.get_logger().info(
        ###    f'front={front_distance:.3f} m, points={front_points} | '
        ###    f'closest={closest_distance:.3f} m @ {closest_angle:.1f} deg'
       ###)

    def destroy_node(self):
        self.running = False

        try:
            if hasattr(self, 'serial') and self.serial is not None:
                self.serial.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LD06Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()