#! /usr/bin/env python

import rclpy
from rclpy.node import Node
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
)
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import argparse
from uwtec_nav.utils.gps_utils import (
    distance_and_bearing,
    calc_goal_heading,
    rotate_to_go,
    coordinate_after_move,
)
import sys
import math


class NavDemo(Node):
    def __init__(self, interval, longitude, latitude, dst_lon, dst_lat, heading, debug):
        super().__init__("nav_demo_node")
        self.interval = interval
        self.longitude = longitude
        self.latitude = latitude
        self.dst_lon = dst_lon
        self.dst_lat = dst_lat
        self.heading = heading
        self.debug = debug

        self.vel_east = 0
        self.vel_north = 0
        self.yaw = 0
        self.offset = 0

        self.start_countdown = 5
        self.ticks = 0

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )
        self.utmpos = self.create_subscription(
            Odometry, "/gps/utmpos", self.utm_callback, 1
        )
        self.gyro_subscriber = self.create_subscription(
            Imu, "/gyro/imu", self.gyro_imu_callback, 1
        )

        self.diff_drive_cmd_vel_publisher = self.create_publisher(
            # TwistStamped, "/diff_drive_base_controller/cmd_vel", 10
            TwistStamped,
            "/cmd_vel_nav",
            10,
        )
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude

    def utm_callback(self, msg):
        self.vel_east = msg.twist.twist.linear.x
        self.vel_north = msg.twist.twist.linear.y
        # self.get_logger().info(f"Vel(east): {vel_east}, Vel(north): {vel_north}")

    def gyro_imu_callback(self, msg):
        quaternion = msg.orientation
        # print(quaternion)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.yaw = math.degrees(yaw) % 360  # rad to deg
        if self.debug:
            print(f"Gyro: {self.yaw:.2f}")

    def timer_callback(self):
        current_heading = (self.yaw - self.offset) % 360
        if self.start_countdown > 0:
            if self.ticks == 0:
                self.get_logger().info(f"{self.start_countdown}...")

            self.ticks += 1
            if self.ticks == int(1 / self.interval):
                self.ticks = 0
                self.start_countdown -= 1
                if self.start_countdown == 0:
                    self.offset = (self.yaw - self.heading) % 360

        else:
            # point1 = (self.latitude, self.longitude)
            point1 = coordinate_after_move(
                self.latitude,
                self.longitude,
                self.vel_east,
                self.vel_north,
                self.interval,
            )
            point2 = (self.dst_lat, self.dst_lon)
            (distance, bearing) = distance_and_bearing(point1, point2)
            degree = rotate_to_go(current_heading, bearing)
            self.ticks += 1
            if self.ticks == int(1 / self.interval):
                self.ticks = 0
                self.get_logger().info(
                    f"\nSource: {point1}\nTarget: {point2} \
                    \nDistance: {distance:.2f}\nBearing: {bearing:.2f} \nHeading: {current_heading:.2f} \
                    \n=> {degree:.2f} deg\n"
                )
            # sys.exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=0.1, help="timer interval")
    ap.add_argument(
        "--longitude", type=float, default=127.525561, help="initial longitude"
    )
    ap.add_argument(
        "--latitude", type=float, default=37.719517, help="initial latitude"
    )
    ap.add_argument(
        "--dst_lon", type=float, default=127.52545426, help="target longitude"
    )
    ap.add_argument(
        "--dst_lat", type=float, default=37.71942289, help="target latitude"
    )
    # ap.add_argument(
    #     "--dst_lon", type=float, default=127.5254894, help="target longitude"
    # )
    # ap.add_argument("--dst_lat", type=float, default=37.719517, help="target latitude")
    ap.add_argument(
        "--heading", type=float, default=0.0, help="initial heading for testing"
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )
    args = vars(ap.parse_args())
    print(args)

    rclpy.init()
    node = NavDemo(**args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
