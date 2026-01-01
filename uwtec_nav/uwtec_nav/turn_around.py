#! /usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
)

# from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TwistStamped
import argparse
import math
from uwtec_nav.utils.heading_utils import calc_goal_heading, rotate_to_go


class NavDemo(Node):
    def __init__(self, interval, degree, accuracy, angular, debug, heading):
        super().__init__("nav_demo_node")
        self.interval = interval
        self.degree = degree
        self.accuracy = accuracy
        self.angular = angular
        self.debug = debug

        self.goal_heading = 0.0
        self.start_countdown = 5

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = heading
        self.yaw = 0
        self.offset = 0

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
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

    def timer_callback(self):
        current_heading = (self.yaw - self.offset) % 360
        if self.start_countdown > 0:
            self.get_logger().info(f"{self.start_countdown}...")
            self.start_countdown -= 1
            if self.start_countdown == 0:
                self.goal_heading = calc_goal_heading(self.heading, self.degree)
                self.offset = (self.yaw - self.heading) % 360

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

    def gyro_imu_callback(self, msg):
        quaternion = msg.orientation
        # print(quaternion)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.yaw = math.degrees(yaw) % 360  # rad to deg
        if self.debug:
            print(f"Gyro: {self.yaw:.2f}")

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
    ap.add_argument(
        "--degree", type=int, default=90, help="turning direction by degree"
    )
    ap.add_argument("--heading", type=float, default=0.0, help="initial heading")
    ap.add_argument("--accuracy", type=int, default=5, help="target accuracy")
    ap.add_argument(
        "-a", "--angular", type=float, default=0.5, help="angular velocity (-1 ~ 1)"
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
