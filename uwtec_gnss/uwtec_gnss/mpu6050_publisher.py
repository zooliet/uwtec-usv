#! /usr/bin/env python

import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import NavSatFix
# from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from uwtec_gnss.mpu6050_driver import MPU6050Driver

import argparse
import serial
import sys
import math


class GyroNode(Node):
    def __init__(self, port, baudrate, debug):
        super().__init__("gyro_threaded_node")
        self.debug = debug

        # Create and start a separate thread
        try:
            self.gyro_driver = MPU6050Driver(port, baudrate, False)
            self.get_logger().info("Gyro driver initialized successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize Gyro driver: {e}")
            sys.exit(1)
        self.gyro_driver.start()

        # pub topics
        self.gyro_pub = self.create_publisher(Imu, "/gyro/imu", 1)

        # timer for every 1/2 sec
        self.pub_timer = self.create_timer(0.1, self.pub_task)

    def pub_task(self):
        heading = self.gyro_driver.heading

        this_time = self.get_clock().now().to_msg()

        # Publish IMU Data
        imu_msg = Imu()
        imu_msg.header.stamp = this_time
        imu_msg.header.frame_id = "gyro_link"
        quaternion = quaternion_from_euler(
            # math.radians(roll), math.radians(pitch), math.radians(heading)
            0.0,
            0.0,
            math.radians(heading),
        )
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        self.gyro_pub.publish(imu_msg)

        # Test
        if self.debug:
            _, _, yaw = euler_from_quaternion(quaternion)
            yaw = math.degrees(yaw) % 360  # rad to deg
            self.get_logger().info(f"Heading: {heading:.2f}, {yaw:.2f}")

    def stop(self):
        print("Stopping Gyro Node...")
        self.gyro_driver.stop()


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="/dev/ttyAMA0", help="Gyro device path")
    ap.add_argument(
        "-b", "--baudrate", type=int, default=115200, help="current baudrate"
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )

    args = vars(ap.parse_args())
    print(args)
    node = GyroNode(**args)
    try:
        rclpy.spin(node)  # Spin the main thread for ROS 2 callbacks
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.gyro_driver.join()  # Wait for the separate thread to finish
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
