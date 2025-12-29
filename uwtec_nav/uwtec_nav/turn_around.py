#! /usr/bin/env python

# import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu

# from nav_msgs.msg import Odometry
# from uwtec_nav.utils.gps_utils import (
#     euler_from_quaternion,
# )
from geometry_msgs.msg import TwistStamped
import argparse
import math
from uwtec_nav.utils.heading_utils import calc_goal_heading, rotate_to_go


class NavDemo(Node):
    def __init__(self, params):
        super().__init__("nav_demo_node")
        self.interval = float(params.get("interval", 1.0))
        self.duration = int(params.get("duration", 30))
        self.angular = float(params.get("angular", 0.5))
        self.degree = int(params.get("degree", 90))
        self.accuracy = int(params.get("accuracy", 5))

        self.goal_heading = 0.0
        self.start_count = 3

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = float(params.get("heading", 0))

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )

        self.diff_drive_cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/diff_drive_base_controller/cmd_vel", 10
        )
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        current_heading = self.heading
        if self.start_count > 0:
            self.goal_heading = calc_goal_heading(current_heading, self.degree)
            self.start_count -= 1
        else:
            degree = rotate_to_go(current_heading, self.goal_heading)
            self.get_logger().info(
                f"\nHeading: {current_heading:.2f}, Goal: {self.goal_heading}\t=>\t{degree}"
            )

            if math.fabs(degree) > self.accuracy:
                self.turn_around(degree)
            else:
                self.get_logger().info("Goal Achieved.")
                exit(0)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude

    def turn_around(self, degree):
        sign = 1 if degree > 0 else -1
        degree = math.fabs(degree)
        if degree > 20:
            angular_speed = self.angular * sign
        elif degree > 10:
            angular_speed = 0.3 * sign
        elif degree > self.accuracy:
            angular_speed = 0.2 * sign
        else:
            angular_speed = 0.0

        # self.get_logger().info(f"\nAngular speed: {angular_speed}")
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.angular.z = angular_speed
        self.diff_drive_cmd_vel_publisher.publish(twist_msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="timer interval")
    # ap.add_argument("-d", "--duration", type=int, default=30, help="duration")
    ap.add_argument(
        "--degree", type=int, default=90, help="turning direction by degree"
    )
    ap.add_argument("--heading", type=float, default=0.0, help="initial heading")
    ap.add_argument("--accuracy", type=int, default=5, help="target accuracy")
    ap.add_argument(
        "-a", "--angular", type=float, default=0.5, help="angular velocity (-1 ~ 1)"
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
