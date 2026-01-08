#! /usr/bin/env python

# import sys
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import NavSatFix, Imu
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
)
from geometry_msgs.msg import TwistStamped
import argparse
import math
from uwtec_nav.utils.gps_utils import (
    distance_and_bearing,
    calc_goal_heading,
    rotate_to_go,
)

# from enum import Enum
# class Status(Enum):
#     STOP = 1
#     TURN = 2
#     RUN = 3
#     FINISHED = 4


class NavDemo(Node):
    def __init__(
        self,
        interval,
        yaml_file,
        heading,
        angular,
        linear,
        angular_accuracy,
        distance_accuracy,
        debug,
    ):
        super().__init__("nav_demo_node")
        self.interval = interval
        self.yaml_file = yaml_file
        self.angular = angular
        self.linear = linear
        self.angular_accuracy = angular_accuracy
        self.distance_accuracy = distance_accuracy

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = heading
        self.yaw = 0
        self.offset = 0
        self.debug = debug
        self.start_countdown = 5
        self.bearing_goal = None

        self.coords = []
        yaml_file_path = os.path.join(
            get_package_share_directory("uwtec_nav"), "config", self.yaml_file
        )
        with open(yaml_file_path, "r") as wps_file:
            self.coords = yaml.safe_load(wps_file)

        # self.status = Status.STOP
        self.wps_index = 0

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

    def timer_callback(self):
        current_heading = (self.yaw - self.offset) % 360
        if self.start_countdown > 0:
            self.get_logger().info(f"{self.start_countdown}...")
            self.start_countdown -= 1
            if self.start_countdown == 0:
                self.offset = (self.yaw - self.heading) % 360
        else:
            point1 = (self.latitude, self.longitude)
            coords = self.coords[self.wps_index]
            point2 = (coords["Lat"], coords["Lon"])
            # self.get_logger().info(f"\nSource: {point1}\nTarget: {point2}\n")
            (distance, bearing) = distance_and_bearing(point1, point2)
            if self.bearing_goal is None:
                self.bearing_goal = bearing
                degree = rotate_to_go(current_heading, self.bearing_goal)
                self.get_logger().info(
                    f"\nSource: {point1}\nTarget: {point2} \
                    \nDistance: {distance:.2f}\nHeading: {current_heading:.2f} \
                    \n=> {degree:.2f} deg\n"
                )

            degree = rotate_to_go(current_heading, self.bearing_goal)
            if math.fabs(degree) > self.angular_accuracy:  # * 2.0:
                self.turn_around(degree)
            else:
                print(f"Bearing Finished: {self.wps_index}")
                self.bearing_goal = None
                self.wps_index += 1
                if self.wps_index == len(self.coords):
                    self.get_logger().info("Navigation Finished.")
                    self.timer.cancel()
                    exit(0)

            # if math.fabs(distance) > self.distance_accuracy:
            #     if math.fabs(degree) < self.angular_accuracy:
            #         degree = 0
            #     self.go_drive(distance, degree)
            # #     if math.fabs(degree) > self.angular_accuracy * 2.0:
            # #         self.turn_around(degree)
            # #     else:
            # #         self.go_drive(distance, degree)
            # else:
            #     print(f"WP #{self.wps_index} Finished")
            #     self.bearing_goal = None
            #     self.wps_index += 1
            #     if self.wps_index == len(self.coords):
            #         self.get_logger().info("Navigation Finished.")
            #         self.timer.cancel()
            #         exit(0)

    def go_drive(self, distance, degree):
        sign = -1 if degree > 0 else 1
        degree = math.fabs(degree)
        if degree == 0:
            angular_speed = 0
        else:
            angular_speed = self.angular * sign
        # if degree > 20:
        #     angular_speed = self.angular * sign
        # elif degree > 10:
        #     angular_speed = 0.3 * sign
        # elif degree > self.angular_accuracy:
        #     angular_speed = 0.2 * sign
        # else:
        #     angular_speed = 0.0

        distance = math.fabs(distance)
        linear_speed = self.linear
        # if distance > 20:
        #     linear_speed = self.linear
        # elif distance > 10:
        #     linear_speed = 0.3
        # elif distance > self.distance_accuracy:
        #     linear_speed = 0.2
        # else:
        #     linear_speed = 0.0

        self.get_logger().info(
            f"\nAngular speed: {angular_speed}\nLinear speed: {linear_speed}"
        )
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.linear.x = linear_speed
        twist_msg.twist.angular.z = angular_speed
        self.diff_drive_cmd_vel_publisher.publish(twist_msg)

    def turn_around(self, degree):
        sign = -1 if degree > 0 else 1
        degree = math.fabs(degree)
        angular_speed = self.angular * sign
        # if degree > 20:
        #     angular_speed = self.angular * sign
        # elif degree > 10:
        #     angular_speed = 0.3 * sign
        # elif degree > self.angular_accuracy:
        #     angular_speed = 0.2 * sign
        # else:
        #     angular_speed = 0.0

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
        "-y", "--yaml-file", default="uwtec_wps.yaml", help="waypoints file"
    )
    ap.add_argument(
        "--heading", type=float, default=0.0, help="initial heading for testing"
    )
    ap.add_argument(
        "--distance-accuracy", type=int, default=5, help="target distance accuracy"
    )
    ap.add_argument(
        "--angular-accuracy", type=int, default=3, help="target angular accuracy"
    )
    ap.add_argument(
        "-a", "--angular", type=float, default=0.4, help="angular velocity (-1 ~ 1)"
    )
    ap.add_argument(
        "-l", "--linear", type=float, default=0.4, help="linear velocity (-1 ~ 1)"
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
