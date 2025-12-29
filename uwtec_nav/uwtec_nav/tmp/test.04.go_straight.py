#! /usr/bin/env python

import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from uwtec_nav.utils.gps_utils import (
    euler_from_quaternion,
)
from geometry_msgs.msg import TwistStamped

class GPSDemo(Node):
    def __init__(self, duration):
        super().__init__("gps_demo_node")

        self.duration = duration

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
        self.get_logger().info(
            f"Current GPS data - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f}"
        )

        if self.duration > 0:
            self.go_straight()
            self.duration -= 1

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude

    def go_straight(self):
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.linear.x = 0.5
        # twist_msg.twist.angular.z = 0.5
        self.diff_drive_cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        duration = int(sys.argv[1])
    else:
        duration = 10 

    node = GPSDemo(duration)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
