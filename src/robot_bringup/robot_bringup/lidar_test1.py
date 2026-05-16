import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarSectorTest(Node):
    def __init__(self):
        super().__init__('lidar_sector_test')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/lidar_sectors')
        self.declare_parameter('log_period', 0.5)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.log_period = float(self.get_parameter('log_period').value)

        self.last_log_time = 0.0

        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            self.output_topic,
            10
        )

        self.get_logger().info(
            f'Lidar sector test started. Listening: {self.scan_topic}, '
            f'publishing: {self.output_topic}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def normalize_angle_deg(self, angle_deg):
        return angle_deg % 360.0

    def sector_index(self, angle_deg):
        """
        0 = FRONT  : 315..360 and 0..45
        1 = LEFT   : 45..135
        2 = BACK   : 135..225
        3 = RIGHT  : 225..315
        """
        a = self.normalize_angle_deg(angle_deg)

        if a >= 315.0 or a < 45.0:
            return 0
        elif 45.0 <= a < 135.0:
            return 1
        elif 135.0 <= a < 225.0:
            return 2
        else:
            return 3

    def sector_name(self, index):
        names = {
            0: 'FRONT',
            1: 'LEFT',
            2: 'BACK',
            3: 'RIGHT',
        }
        return names.get(index, 'UNKNOWN')

    def min_stable_distance(self, values):
        if not values:
            return None

        values.sort()

        # 10% точка, щоб не брати одиничний шум
        idx = max(0, min(len(values) - 1, int(len(values) * 0.1)))
        return values[idx]

    def scan_callback(self, msg: LaserScan):
        now = self.now_sec()

        if now - self.last_log_time < self.log_period:
            return

        self.last_log_time = now

        sectors = {
            0: [],
            1: [],
            2: [],
            3: [],
        }

        angle = msg.angle_min
        valid_points = 0

        for i, r in enumerate(msg.ranges):
            angle_deg = math.degrees(angle)
            angle_deg = self.normalize_angle_deg(angle_deg)

            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                idx = self.sector_index(angle_deg)
                sectors[idx].append((i, angle_deg, r))
                valid_points += 1

            angle += msg.angle_increment

        lines = []
        lines.append('===== LIDAR SECTOR INDEX TEST =====')
        lines.append(f'angle_min={math.degrees(msg.angle_min):.1f} deg')
        lines.append(f'angle_max={math.degrees(msg.angle_max):.1f} deg')
        lines.append(f'angle_increment={math.degrees(msg.angle_increment):.2f} deg')
        lines.append(f'total_points={len(msg.ranges)}')
        lines.append(f'valid_points={valid_points}')

        for idx in range(4):
            sector_data = sectors[idx]
            distances = [item[2] for item in sector_data]
            min_dist = self.min_stable_distance(distances)

            if sector_data:
                first_point_index = sector_data[0][0]
                last_point_index = sector_data[-1][0]
                first_angle = sector_data[0][1]
                last_angle = sector_data[-1][1]

                if min_dist is not None:
                    line = (
                        f'[{idx}] {self.sector_name(idx):5s} | '
                        f'points={len(sector_data):3d} | '
                        f'index_range={first_point_index}..{last_point_index} | '
                        f'angle_range={first_angle:.1f}..{last_angle:.1f} deg | '
                        f'min={min_dist:.3f} m'
                    )
                else:
                    line = (
                        f'[{idx}] {self.sector_name(idx):5s} | '
                        f'points={len(sector_data):3d} | '
                        f'NO VALID DIST'
                    )
            else:
                line = (
                    f'[{idx}] {self.sector_name(idx):5s} | '
                    f'points=0 | NO DATA'
                )

            lines.append(line)

        text = '\n'.join(lines)

        self.get_logger().info('\n' + text)

        out = String()
        out.data = text
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSectorTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()