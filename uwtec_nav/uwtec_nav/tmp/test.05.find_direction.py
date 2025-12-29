#! /usr/bin/env python

import sys
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
    distance_and_bearing
)
from geometry_msgs.msg import TwistStamped

class GPSDemo(Node):
    def __init__(self, lat, lon):
        super().__init__("gps_demo_node")

        self.target_point = (lat, lon)
        # self.heading_goal = 0

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/custom", self.gps_custom_callback, 1
        )

        self.diff_drive_cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/diff_drive_base_controller/cmd_vel", 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):

        starting_point = (self.latitude, self.longitude)
        _, bearing = distance_and_bearing(starting_point, self.target_point)

        heading_diff = self.heading - bearing
        self.get_logger().info(
            f"\nHeading: {self.heading:.2f}, Bearing: {bearing:.2f}, Diff: {heading_diff:.2f}"
        )

        if math.fabs(heading_diff) > 5:
            self.go_turning(heading_diff)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude

    def go_turning(self, heading_diff):
        self.get_logger().info(f"{heading_diff}")
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        # twist_msg.twist.linear.x = 0.5
        sign = 1 if heading_diff > 0 else -1
        heading_diff = math.fabs(heading_diff)
        twist_msg.twist.angular.z = 0.1*sign

        # if heading_diff > 20:
        #     twist_msg.twist.angular.z = 1.0*sign
        # elif heading_diff > 10:  
        #     twist_msg.twist.angular.z = 0.5*sign
        # elif heading_diff > 5:  
        #     twist_msg.twist.angular.z = 0.3*sign
        # else:
        #     twist_msg.twist.angular.z = 0

        self.diff_drive_cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 2:
        lat = float(sys.argv[1]) 
        lon = float(sys.argv[2]) 
    else:
        lat = 37.719468
        lon = 127.525408

    node = GPSDemo(lat, lon)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
