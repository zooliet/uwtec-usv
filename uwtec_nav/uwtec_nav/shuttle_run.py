#! /usr/bin/env python

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
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
    coordinate_after_move,
)

import sys
import os
import yaml

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
        runs,
        yaml_file,
        angular,
        linear,
        angular_accuracy,
        distance_accuracy,
        heading,
        skip,
        debug,
    ):
        super().__init__("nav_demo_node")
        self.interval = interval
        self.runs = runs
        self.yaml_file = yaml_file
        self.angular = angular
        self.linear = linear
        self.angular_accuracy = angular_accuracy
        self.distance_accuracy = distance_accuracy
        self.heading = heading
        self.skip = skip
        self.debug = debug

        self.latitude = 0.0
        self.longitude = 0.0
        self.vel_east = 0
        self.vel_north = 0
        self.yaw = 0
        self.offset = 0
        self.bearing = None

        self.start_countdown = 5
        self.ticks = 0
        self.run = 0
        self.dummy_count = 50  # 의도: 5 sec

        self.coords = []
        yaml_file_path = os.path.join(
            get_package_share_directory("uwtec_nav"), "config", self.yaml_file
        )
        with open(yaml_file_path, "r") as wps_file:
            self.coords = yaml.safe_load(wps_file)

        # 이 app 에서는 1개의 entry 만 필요함
        while len(self.coords) > 1:
            self.coords.pop()

        # self.status = Status.STOP
        self.wps_index = 0

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
                    self.coords.append({"Lat": self.latitude, "Lon": self.latitude})

        else:
            # point1 = (self.latitude, self.longitude)
            point1 = coordinate_after_move(
                self.latitude,
                self.longitude,
                self.vel_east,
                self.vel_north,
                self.interval,
            )
            coords = self.coords[self.wps_index]
            point2 = (coords["Lat"], coords["Lon"])

            self.shuttle_run(point1, point2, current_heading)

    def shuttle_run(self, point1, point2, current_heading):
        (distance, bearing) = distance_and_bearing(point1, point2)
        if self.skip is True:  # upated point1 을 사용하지 않음
            if self.bearing is None:
                self.bearing = bearing
        else:
            self.bearing = bearing

        degree = rotate_to_go(current_heading, self.bearing)
        self.ticks += 1
        if self.ticks == int(1 / self.interval):
            self.ticks = 0
            self.get_logger().info(
                f"\nSource: {point1}\nTarget: {point2} \
                \nDistance: {distance:.2f}\nBearing: {self.bearing:.2f} \nHeading: {current_heading:.2f} \
                \n=> {degree:.2f} deg\n"
            )

        if math.fabs(distance) > self.distance_accuracy:
            # bearing 이x10 이상(예, 30도) 벌어지면 turn_around() 만 수행
            if math.fabs(degree) > self.angular_accuracy * 10.0:
                self.turn_around(degree)
            elif math.fabs(degree) <= self.angular_accuracy:
                self.go_drive(distance, 0.0)
            else:
                self.go_drive(distance, degree)

        # if self.dummy_count > 0:
        #     self.dummy_count -= 1
        else:
            print(f"WP #{self.wps_index} Finished ({self.run + 1}/{self.runs})")
            self.stop_moving()
            self.bearing = None
            self.wps_index = 1 if self.wps_index == 0 else 0
            self.start_countdown = 5
            self.dummy_count = 50
            self.run += 1
            if self.run == self.runs:
                for coord in self.coords:
                    print(coord)
                self.timer.cancel()
                self.get_logger().info("Navigation Finished.")
                sys.exit(0)

    def go_drive(self, distance, degree):
        sign = -1 if degree > 0.0 else 1
        # degree = math.fabs(degree)
        if degree == 0.0:
            angular_speed = 0.0
        else:
            angular_speed = self.angular / 4 * sign  # need to tune

        distance = math.fabs(distance)
        linear_speed = self.linear

        # self.get_logger().info(
        #     f"\nAngular speed: {angular_speed}\nLinear speed: {linear_speed}"
        # )
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

        # self.get_logger().info(f"\nAngular speed: {angular_speed}")
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.angular.z = angular_speed
        self.diff_drive_cmd_vel_publisher.publish(twist_msg)

    def stop_moving(self):
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.angular.z = 0.0
        self.diff_drive_cmd_vel_publisher.publish(twist_msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=0.1, help="timer interval")
    ap.add_argument("-r", "--runs", type=int, default=4, help="number of runs")
    ap.add_argument(
        "-y", "--yaml-file", default="uwtec_wps.yaml", help="waypoints file"
    )
    ap.add_argument(
        "-a", "--angular", type=float, default=0.4, help="angular velocity (-1 ~ 1)"
    )
    ap.add_argument(
        "-l", "--linear", type=float, default=0.4, help="linear velocity (-1 ~ 1)"
    )
    ap.add_argument(
        "--angular-accuracy", type=int, default=3, help="target angular accuracy"
    )
    ap.add_argument(
        "--distance-accuracy", type=int, default=2, help="target distance accuracy"
    )
    ap.add_argument(
        "--heading", type=float, default=0.0, help="initial heading for testing"
    )
    ap.add_argument(
        "--skip",
        action="store_false",
        help="Skip coord update based on distance(default: True)",
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
