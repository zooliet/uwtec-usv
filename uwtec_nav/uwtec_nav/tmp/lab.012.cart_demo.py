#! /usr/bin/env python

import rclpy
from rclpy.node import Node

import math

from sensor_msgs.msg import NavSatFix, Imu
from uwtec_cart.utils.gps_utils import (
    distance_and_bearing,
)

from geometry_msgs.msg import Twist, Vector3

from geometry_msgs.msg import TwistStamped


class GPSDemo(Node):
    def __init__(self):
        super().__init__("gps_demo_node")

        ## current position and heading
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.target_latitude = 0.0
        self.target_longitude = 0.0001

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )
        self.timer = self.create_timer(1, self.timer_callback)


    def timer_callback(self):
        self.get_logger().info(
            f"Current GPS - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f} radians"
        )
        # d, b = distance_and_bearing((37.719521, 127.525556), (37.719521, 127.525656))
        distance, bearing = distance_and_bearing(
            (self.latitude, self.longitude), 
            (self.target_latitude, self.target_longitude)
        )
        self.get_logger().info(
            f"Distance: {distance:.2f}, bearing: {bearing:.2f}"
        )
        self.go_drive(distance, bearing, self.heading)



    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.heading = msg.altitude
    #     self.get_logger().info(
    # f"Received GPS data - Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, Heading: {self.heading:.2f}"
    #     )
    #
    def go_drive(self, distance, bearing, heading):
        self.rotate(bearing, heading)
        # if distance < 0.5:
        #     self.stop()
        # elif math.fabs(bearing, heading) > 30.0:
        #     self.stop()
        #     self.rotate(bearing, heading)
        # else:
        #     self.navigate(bearing, heading)

    def stop(self):
        msg = Twist()
        msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        # msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(msg)

    def rotate(self, bearing, heading):
        msg = Twist()
        if(bearing > heading):
            msg.linear = Vector3(x=0.2, y=0.0, z=0.0)
            # msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        else:
            msg.linear = Vector3(x=0.0, y=0.2, z=0.0)
        self.cmd_vel_publisher.publish(msg)

    def slow(self):
        msg = Twist()
        msg.linear = Vector3(x=0.1, y=0.1, z=0.0)
        self.cmd_vel_publisher.publish(msg)

    def navigate(self, bearing, heading):
        msg = Twist()
        msg.linear = Vector3(x=0.2, y=0.2, z=0.0)
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
