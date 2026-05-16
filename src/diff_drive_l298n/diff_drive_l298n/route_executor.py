#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import yaml
import sys
import time
import signal


class RouteExecutor(Node):

    def __init__(self, route_file):

        super().__init__("route_executor")

        self.get_logger().info("Route Executor started")

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Load route
        self.load_route(route_file)

        # Ctrl+C handler
        signal.signal(signal.SIGINT, self.stop_robot)

        # Start route
        self.run_route()


    # ---------------- LOAD YAML ----------------

    def load_route(self, file):

        with open(file, "r") as f:
            data = yaml.safe_load(f)

        self.loop = data.get("loop", False)
        self.route = data["route"]

        self.get_logger().info(
            f"Loaded {len(self.route)} steps | loop={self.loop}"
        )


    # ---------------- EXEC STEP ----------------

    def execute_step(self, step):

        msg = Twist()

        msg.linear.x  = float(step["linear"][0])
        msg.linear.y  = float(step["linear"][1])
        msg.linear.z  = float(step["linear"][2])

        msg.angular.x = float(step["angular"][0])
        msg.angular.y = float(step["angular"][1])
        msg.angular.z = float(step["angular"][2])

        mode = step["mode"]

        self.get_logger().info(f"Step: {step['name']} ({mode})")


        # -------- TIME MODE --------
        if mode == "time":

            duration = float(step["duration"])
            start = time.time()

            while time.time() - start < duration:

                self.cmd_pub.publish(msg)
                time.sleep(0.05)


        # -------- ONCE MODE --------
        elif mode == "once":

            self.cmd_pub.publish(msg)
            time.sleep(0.3)


    # ---------------- MAIN LOOP ----------------

    def run_route(self):

        time.sleep(1.0)

        while rclpy.ok():

            for step in self.route:

                self.execute_step(step)

            if not self.loop:
                break

        self.stop_motion()
        self.get_logger().info("Route finished")


    # ---------------- STOP ----------------

    def stop_motion(self):

        msg = Twist()
        self.cmd_pub.publish(msg)


    def stop_robot(self, sig, frame):

        self.get_logger().warn("Stopping robot")

        self.stop_motion()
        rclpy.shutdown()
        sys.exit(0)



# ================= MAIN =====================

def main():

    rclpy.init()

    if len(sys.argv) < 2:

        print("Usage:")
        print("ros2 run route_executor route_node.py route.yaml")
        return

    route_file = sys.argv[1]

    node = RouteExecutor(route_file)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
