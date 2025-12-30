#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu

from geometry_msgs.msg import TwistStamped
import argparse
from uwtec_nav.utils.heading_utils import calc_goal_heading, rotate_to_go
from uwtec_nav.utils.gps_utils import distance_and_bearing


class NavDemo(Node):
    def __init__(self, params):
        super().__init__("nav_demo_node")
        self.interval = float(params.get("interval", 1.0))
        self.target_latitude = float(params.get("target_lat", 0.0))
        self.target_longitude = float(params.get("target_lon", 0.0))

        self.goal_heading = 0.0
        self.start_count = 3

        self.latitude = float(params.get("source_lat", 0.0))
        self.longitude = float(params.get("source_lon", 0.0))
        self.heading = float(params.get("heading", 0))

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )

        self.diff_drive_cmd_vel_publisher = self.create_publisher(
            # TwistStamped, "/diff_drive_base_controller/cmd_vel", 10
            TwistStamped,
            "/cmd_vel_nav",
            10,
        )
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        if self.start_count > 0:
            # wait for 3 seconds
            self.start_count -= 1
        else:
            point1 = (self.latitude, self.longitude)
            point2 = (self.target_latitude, self.target_longitude)
            self.get_logger().info(f"\nSource: {point1}\nTarget: {point2}\n")
            (distance, bearing) = distance_and_bearing(point1, point2)
            degree = rotate_to_go(self.heading + 360, bearing + 360)
            self.get_logger().info(
                f"\nSource: {point1}\nTarget: {point2} \
                \nDistance: {distance:.2f}\nBearing: {bearing:.2f} \nHeading: {self.heading:.2f} \
                \n=> {degree:.2f} deg\n"
            )
            exit(0)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="timer interval")
    ap.add_argument("--heading", type=float, default=0.0, help="initial heading")
    ap.add_argument(
        "--source_lon", type=float, default=127.525561, help="initial longitude"
    )
    ap.add_argument(
        "--source_lat", type=float, default=37.719517, help="initial latitude"
    )
    ap.add_argument(
        "--target_lon", type=float, default=127.525396, help="target longitude"
    )
    ap.add_argument(
        "--target_lat", type=float, default=37.719517, help="target latitude"
    )
    args = vars(ap.parse_args())
    print(args)

    rclpy.init()
    node = NavDemo(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
