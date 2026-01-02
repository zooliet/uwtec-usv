#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
)
from geometry_msgs.msg import TwistStamped
import argparse
import math


class NavDemo(Node):
    def __init__(self, interval, duration, angular, debug):
        super().__init__("nav_demo_node")
        self.interval = interval
        self.duration = duration
        self.angular = angular
        self.debug = debug

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0
        self.yaw = 0
        self.offset = 0
        self.start_countdown = 5

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )
        self.gyro_subscriber = self.create_subscription(
            Imu, "/gyro/imu", self.gyro_imu_callback, 1
        )

        self.diff_drive_cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            # TwistStamped, "/diff_drive_base_controller/cmd_vel", 10
            "/cmd_vel_nav",
            10,
        )
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        current_heading = (self.yaw - self.offset) % 360
        if self.start_countdown > 0:
            self.get_logger().info(f"{self.start_countdown}...")
            self.start_countdown -= 1
            if self.start_countdown == 0:
                self.offset = (self.yaw - self.heading) % 360
        else:
            if self.duration > 0:
                self.go_dancing()
                self.duration -= 1
            self.get_logger().info(
                f"\nHeading: {self.heading:.2f}, Yaw: {current_heading:.2f}, Angular: {self.angular:.2f}"
            )

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude

    def gyro_imu_callback(self, msg):
        quaternion = msg.orientation
        # print(quaternion)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.yaw = math.degrees(yaw) % 360  # rad to deg
        if self.debug:
            print(f"Gyro: {self.yaw:.2f}")

    def go_dancing(self):
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.angular.z = 1.0
        self.diff_drive_cmd_vel_publisher.publish(twist_msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="timer interval")
    ap.add_argument("-d", "--duration", type=int, default=30, help="duration")
    ap.add_argument(
        "-a", "--angular", type=float, default=0.5, help="angular velocity (-1 ~ 1)"
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )
    args = vars(ap.parse_args())
    print(**args)

    rclpy.init()
    node = NavDemo(**args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
