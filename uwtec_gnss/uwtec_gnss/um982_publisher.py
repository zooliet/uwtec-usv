#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from uwtec_gnss.um982_driver import UM982Driver
from uwtec_gnss.utils.nmea_util import create_utm_trans, utm_trans

import argparse
import serial
import sys
import math
import numpy as np


class GNSSNode(Node):
    def __init__(self, port, baudrate, debug):
        super().__init__("gnss_threaded_node")
        self.debug = debug

        # Create and start a separate thread
        try:
            self.gnss_driver = UM982Driver(port, baudrate, False)
            self.get_logger().info("GNSS driver initialized successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize GNSS driver: {e}")
            sys.exit(1)
        self.gnss_driver.start()

        # 서울 시청 좌표를 사용해서 UTM transformer 초기화
        self.transformer = create_utm_trans(36.5665, 127.9780)

        # pub topics
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/custom", 1)
        # self.fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 1)
        self.utm_pub = self.create_publisher(Odometry, "/gps/utmpos", 10)
        # self.imu_pub = self.create_publisher(Imu, "/imu/data", 1)

        # timer for every 1/2 sec
        self.pub_timer = self.create_timer(0.5, self.pub_task)

    def pub_task(self):
        (hgt, lat, lon, hgtstd, latstd, lonstd) = np.mean(
            self.gnss_driver.fixes, axis=0
        )
        (heading, pitch, roll) = np.mean(self.gnss_driver.orientations, axis=0)
        (vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std) = (
            np.mean(self.gnss_driver.velocities, axis=0)
        )
        (utm_x, utm_y) = utm_trans(self.transformer, lon, lat)

        # check the accumulation size
        if self.debug:
            print(f"PVTSLNA - {self.gnss_driver.fixes.shape}")
            print(f"GNHPR - {self.gnss_driver.orientations.shape}")
            print(f"BESTNAVA - {self.gnss_driver.velocities.shape}")

        # reinitialize
        self.gnss_driver.fixes = np.empty(shape=(0, 6))
        self.gnss_driver.velocities = np.empty(shape=(0, 6))
        self.gnss_driver.orientations = np.empty(shape=(0, 3))
        this_time = self.get_clock().now().to_msg()

        # Publish GPS Custom Data
        custom_msg = NavSatFix()
        custom_msg.header.stamp = this_time
        custom_msg.header.frame_id = "gps_link"
        custom_msg.latitude = lat
        custom_msg.longitude = lon
        custom_msg.altitude = heading
        self.gps_pub.publish(custom_msg)

        # # Publish GPS Fix Data
        # fix_msg = NavSatFix()
        # fix_msg.header.stamp = this_time
        # fix_msg.header.frame_id = "gps_link"
        # fix_msg.latitude = lat
        # fix_msg.longitude = lon
        # fix_msg.altitude = hgt
        # fix_msg.position_covariance[0] = float(latstd) ** 2
        # fix_msg.position_covariance[4] = float(lonstd) ** 2
        # fix_msg.position_covariance[8] = float(hgtstd) ** 2
        # fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # self.fix_pub.publish(fix_msg)

        # Publish UTM Position Data
        odom_msg = Odometry()
        odom_msg.header.stamp = this_time
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = utm_x
        odom_msg.pose.pose.position.y = utm_y
        odom_msg.pose.pose.position.z = hgt
        quaternion = quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(heading)
        )
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = float(latstd) ** 2
        odom_msg.pose.covariance[7] = float(lonstd) ** 2
        odom_msg.pose.covariance[14] = float(hgtstd) ** 2
        odom_msg.pose.covariance[21] = 0.1
        odom_msg.pose.covariance[28] = 0.1
        odom_msg.pose.covariance[35] = 0.1
        odom_msg.twist.twist.linear.x = vel_east
        odom_msg.twist.twist.linear.y = vel_north
        odom_msg.twist.twist.linear.z = vel_ver
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = float(vel_east_std) ** 2
        odom_msg.twist.covariance[7] = float(vel_north_std) ** 2
        odom_msg.twist.covariance[14] = float(vel_ver_std) ** 2
        self.utm_pub.publish(odom_msg)

        # # Publish IMU Data
        # imu_msg = Imu()
        # imu_msg.header.stamp = this_time
        # imu_msg.header.frame_id = "imu_link"
        # quaternion = quaternion_from_euler(
        #     math.radians(roll), math.radians(pitch), math.radians(heading)
        # )
        # imu_msg.orientation.x = quaternion[0]
        # imu_msg.orientation.y = quaternion[1]
        # imu_msg.orientation.z = quaternion[2]
        # imu_msg.orientation.w = quaternion[3]
        # self.imu_pub.publish(imu_msg)

        # # Test
        # _, _, yaw = euler_from_quaternion(quaternion)
        # self.get_logger().info(f"Heading: {heading}, {yaw}")

    def stop(self):
        print("Stopping GNSS Node...")
        self.gnss_driver.stop()


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="/dev/ttyGNSS", help="GNSS device path")
    ap.add_argument(
        "-b", "--baudrate", type=int, default=115200, help="current baudrate"
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )

    args = vars(ap.parse_args())
    print(args)
    node = GNSSNode(**args)
    try:
        rclpy.spin(node)  # Spin the main thread for ROS 2 callbacks
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.gnss_driver.join()  # Wait for the separate thread to finish
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
