#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory
import math
import os
import sys
import yaml
import time

from sensor_msgs.msg import NavSatFix, Imu
from uwtec_cart.utils.gps_utils import (
    latLonYaw2Geopose,
    euler_from_quaternion,
)


class GPSDemo(Node):
    def __init__(self, wps_file_path):
        super().__init__("gps_demo_node")

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )
        self.imu_subscriber = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, 1
        )
        self.timer = self.create_timer(3.0, self.timer_callback)

        # Initialize the BasicNavigator
        self.navigator = BasicNavigator()
        self.wps_file_path = wps_file_path

    def timer_callback(self):
        self.geopose = latLonYaw2Geopose(self.latitude, self.longitude, self.heading)
        # self.get_logger().info(
        #     f"Current GPS data - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f} radians"
        # )
        # self.get_logger().info(
        #     f"Current GeoPose:\n\tPosition: {self.geopose.position}\n\tOrientation: {self.geopose.orientation}"
        # )

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        # self.get_logger().info(
        #     f"Received GPS data - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}"
        # )

    def imu_callback(self, msg):
        _, _, self.heading = euler_from_quaternion(msg.orientation)
        # self.get_logger().info(f"Received IMU data - Heading: {self.heading:.2f} radians")

    def start_wpf(self):
        self.navigator.waitUntilNav2Active(localizer="robot_localization")
        self.geopose_wps = self.get_waypoints_from_yaml(self.wps_file_path)
        self.get_logger().info(
            f"Loaded {len(self.geopose_wps)} waypoints from YAML file."
        )
        self.get_logger().info("Starting GPS Waypoint Follower...")
        self.navigator.followGpsWaypoints(self.geopose_wps)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        print("wps completed successfully")

    def get_waypoints_from_yaml(self, yaml_file_path):
        # with open(yaml_file_path, "r") as wps_file:
        #     wps_dict = yaml.safe_load(wps_file)

        wps_dict = {
            "waypoints": [
                {"latitude": 38.1614560, "longitude": -122.4546443},
                {"latitude": 38.1614810, "longitude": -122.4546453},
            ]
        }

        start_latitue = self.latitude
        start_longitude = self.longitude
        start_heading = self.heading

        prev_latitude = start_latitue
        prev_longitude = start_longitude

        geopose_wps = []
        for wp in wps_dict["waypoints"]:
            latitude, longitude = wp["latitude"], wp["longitude"]
            yaw = math.atan2((latitude - prev_latitude), (longitude - prev_longitude))
            geopose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
            prev_latitude = latitude
            prev_longitude = longitude

        geopose_wps.append(
            latLonYaw2Geopose(start_latitue, start_longitude, start_heading)
        )
        return geopose_wps


def main(args=None):
    rclpy.init(args=args)

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(
        get_package_share_directory("nav2_gps_waypoint_follower_demo"),
        "config",
        "uwtec_waypoints.yaml",
    )
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GPSDemo(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()
