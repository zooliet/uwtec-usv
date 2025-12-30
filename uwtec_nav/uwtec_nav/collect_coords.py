#! /usr/bin/env python

import sys
import select
import os
import yaml


import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import NavSatFix, Imu

from geometry_msgs.msg import TwistStamped
import argparse
from uwtec_nav.utils.heading_utils import calc_goal_heading, rotate_to_go
from uwtec_nav.utils.gps_utils import distance_and_bearing


class NavDemo(Node):
    def __init__(self, params):
        super().__init__("nav_demo_node")
        self.interval = float(params.get("interval", 1.0))
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.coords = []
        self.yaml_file_path = os.path.join(
            get_package_share_directory("uwtec_nav"), "config", params["yaml"]
        )

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )

        # self.diff_drive_cmd_vel_publisher = self.create_publisher(
        #     # TwistStamped, "/diff_drive_base_controller/cmd_vel", 10
        #     TwistStamped,
        #     "/cmd_vel_nav",
        #     10,
        # )
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        timeout = 1  # seconds

        self.get_logger().info("Enter to collect! | 'q' to save and quit")
        # select returns 3 lists: rlist, wlist, xlist
        # rlist will contain sys.stdin if input is available within the timeout
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            s = sys.stdin.readline().strip()
            if s == "q":
                self.timer.cancel()
                self.save_to_yaml()
            else:
                self.get_logger().info("collected!\n")
                self.coords.append({"Lat": self.latitude, "Lon": self.longitude})

    def save_to_yaml(self):
        # for coord in self.coords:
        #     print(coord)

        with open(self.yaml_file_path, "w") as wps_file:
            yaml.dump(self.coords, wps_file, sort_keys=False)

        self.get_logger().info(f"\nsaved to {self.yaml_file_path} successfully")
        exit(0)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="timer interval")
    ap.add_argument("-y", "--yaml", default="waypoints.yaml", help="waypoints file")
    args = vars(ap.parse_args())
    print(args)

    rclpy.init()
    node = NavDemo(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
