#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
# from ament_index_python.packages import get_package_share_directory
import math
# import yaml
# import os
# import sys
# import time

from sensor_msgs.msg import NavSatFix, Imu
from uwtec_cart.utils.gps_utils import (
    latLonYaw2Geopose,
    euler_from_quaternion,
)

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class GPSDemo(Node):
    def __init__(self):
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
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

        self.keyboard_subscription = self.create_subscription(
            TwistStamped,
            "/cmd_vel_kbd",
            self.keyboard_callback,
            10,
        )

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
        # self.get_logger().info(f"Raw IMU data: {msg}")
        _, _, self.heading = euler_from_quaternion(msg.orientation)
        # self.get_logger().info(f"Received IMU data - Heading: {self.heading:.2f} radians")

    def keyboard_callback(self, msg: TwistStamped):
        # latitude = 38.161491054181276
        # longitude = -122.45464431092836
        # yaw = 0.0
        # wp = [latLonYaw2Geopose(latitude, longitude, yaw)]
        # self.navigator.followGpsWaypoints(wp)
        # if self.navigator.isTaskComplete():
        #     self.get_logger().info("wps completed successfully")
        twist: Twist = msg.twist
        self.get_logger().info(
            f"Received keyboard command - Linear X: {twist.linear.x}, Y: {twist.linear.y}"
        )

        goal_latitude = (
            # self.latitude + twist.linear.y * 0.000025
            self.latitude + twist.linear.y * 0.000050
        )  # Adjust scaling factor as needed
        goal_longitude = (
            # self.longitude + twist.linear.x * 0.000025
            self.longitude + twist.linear.x * 0.000050
        )  # Adjust scaling factor as needed
        goal_heading = math.atan2(
            (goal_latitude - self.latitude), (goal_longitude - self.longitude)
        )
        self.get_logger().info(
            f"Calculated Goal - Latitude: {goal_latitude:.6f}, Longitude: {goal_longitude:.6f}, Heading: {goal_heading:.2f} radians"
        )
        wp = [latLonYaw2Geopose(goal_latitude, goal_longitude, goal_heading)]
        self.navigator.followGpsWaypoints(wp)
        if self.navigator.isTaskComplete():
            self.get_logger().info("wps completed successfully")


def main(args=None):
    rclpy.init(args=args)
    node = GPSDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
