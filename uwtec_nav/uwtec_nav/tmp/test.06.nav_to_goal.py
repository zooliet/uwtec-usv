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
        self.get_logger().info(
            f"Current GPS data - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f}"
        )

        starting_point = (self.latitude, self.longitude)
        distance, heading_diff = distance_and_bearing(starting_point, self.target_point)

        if math.fabs(distance) > 10:
            self.drive_to(distance, heading_diff)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude

    def drive_to(self, distance, heading):
        self.get_logger().info(f"{distance}, {heading}")
        this_time = self.get_clock().now().to_msg()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = this_time
        twist_msg.header.frame_id = "base_footprint"

        if distance > 20:
            twist_msg.twist.linear.x = 0.5
        elif distance > 10:
            twist_msg.twist.linear.x = 0.2
        else:
            twist_msg.twist.linear.x = 0.0

        sign = 1 if heading > 0 else -1
        heading = math.fabs(heading)
        if heading > 20:
            twist_msg.twist.angular.z = 1.0*sign
        elif heading > 10:  
            twist_msg.twist.angular.z = 0.5*sign
        elif heading > 5:  
            twist_msg.twist.angular.z = 0.3*sign
        else:
            twist_msg.twist.angular.z = 0.0

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
