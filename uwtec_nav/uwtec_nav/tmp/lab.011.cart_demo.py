#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import NavSatFix
from uwtec_cart.utils.gps_utils import (
    distance_and_bearing,
)

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class GPSDemo(Node):
    def __init__(self):
        super().__init__("gps_demo_node")

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )
        self.timer = self.create_timer(3.0, self.timer_callback)


    def timer_callback(self):
        self.get_logger().info(
            f"Current GPS - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f} radians"
        )
        d, t, b = distance_and_bearing((37.719521, 127.525556), (37.719521, 127.525656))
        self.get_logger().info(
            f"Distance: {d:.2f}, theta: {t:.2f}, bearing: {b:.2f}"
        )

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude
    #     self.get_logger().info(
    # f"Received GPS data - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f}"
    #     )




def main(args=None):
    rclpy.init(args=args)
    node = GPSDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
