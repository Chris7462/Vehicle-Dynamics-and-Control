#!/usr/bin/env python3
"""
waypoint_collector_ros_only_mwe.py
=================================

What this does:
- Subscribes to the ego vehicle odometry topic published by carla-ros-bridge
- Logs waypoints to a CSV file when you press Ctrl-C

Important:
- This is a minimum working example (MWE).
- The output folder must already exist, or Python will error when writing the file.
- You must edit CSV_FILENAME to your own folder.
"""

import csv
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ------------------------------------------------------------
# EDIT THIS: choose a filepath to save the waypoint data, confirm the path exists (example /home/za/Documents/waypoints.csv)
# ------------------------------------------------------------
CSV_FILENAME = "/workspace/src/waypoint/waypoints.csv"


class WaypointCollector(Node):
    """Stores waypoints in memory and writes them to CSV at shutdown."""

    def __init__(self) -> None:
        super().__init__("waypoint_collector")

        self._rows: list[list[float]] = []

        # ------------------------------------------------------------
        # EDIT THIS: Use "ros2 topic list" to find the odometry topic (something like carla/odometry) and put the topic name here
        # We can use just the odometry topic since it includes pose + velocity
        # ------------------------------------------------------------
        self.create_subscription(Odometry, "/carla/hero/odometry", self._on_odom, 10)

        self.get_logger().info("Waypoint logger running.")
        self.get_logger().info("Drive or enable autopilot, then press Ctrl-C to save waypoints.")
        self.get_logger().info(f"CSV output: {CSV_FILENAME}")

    def _on_odom(self, msg: Odometry) -> None:
        # Time (seconds)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Position (meters)
        p = msg.pose.pose.position

        # Speed (m/s), computed from odometry twist (time-aligned with pose)
        v = msg.twist.twist.linear
        # ------------------------------------------------------------
        # EDIT THIS: choose between longitudinal speed (v.x) or ground speed (the magnitude of all vector components math.sqrt(x^2+y^2+z^2))
        # ------------------------------------------------------------
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)


        # ------------------------------------------------------------
        # EDIT THIS: choose which quantities to measure (pose x-dir is p.x for example, time is probably a good one)
        # ------------------------------------------------------------
        self._rows.append([t, p.x, p.y, p.z, speed])


    def write_csv(self) -> None:
        if not self._rows:
            print("No data collected. Did you start the ROS bridge and spawn the ego vehicle?")
            return

        print(f"Writing {len(self._rows)} rows → {CSV_FILENAME}")
        with open(CSV_FILENAME, "w", newline="") as fp:
            w = csv.writer(fp)
            w.writerow(['time', 'x', 'y', 'z', 'speed']) #name the rows you defined above in "self._rows.append([?, ?, ?, ...])"
            w.writerows(self._rows)

            #What is the sampling frequency? How could you downsample or upsample?

        print("Done. Open waypoints.csv and check that the numbers look reasonable.")


def main() -> None:
    rclpy.init()
    node = WaypointCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
