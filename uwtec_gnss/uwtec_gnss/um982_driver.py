#! /usr/bin/env python

import argparse
import threading
import serial
import time
from uwtec_gnss.utils.nmea_util import (
    nmea_expend_crc,
    nmea_crc,
    PVTSLN_solver,
    GNHPR_solver,
    BESTNAV_solver,
)
import numpy as np


class UM982Driver(threading.Thread):
    def __init__(self, port, baudrate, debug):
        super().__init__()
        self.device = serial.Serial(port, int(baudrate))
        self.running = False
        self.fixes = np.empty(shape=(0, 6))
        self.orientations = np.empty(shape=(0, 3))
        self.velocities = np.empty(shape=(0, 6))
        self.nmea = None
        self.debug = debug
        self.frame_no = 0

    def read_frame(self):
        try:
            frame = self.device.readline().decode("utf-8").strip()
        except Exception as e:
            print(e)
            return

        # if self.debug:
        #     print(f"Received frame: {frame}")

        if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
            try:
                fix = PVTSLN_solver(frame)
                self.fixes = np.vstack((self.fixes, np.array(fix)))
                if self.debug:
                    if self.fixes.shape[0] % 100 == 0:
                        print(f"PVTSLNA - {self.fixes.shape}")

            except Exception as _:
                print(f"PVTSLNA#{self.fixes.shape[0]:04d}, Error")

        elif frame.startswith("$GNHPR") and nmea_crc(frame):
            try:
                orientation = GNHPR_solver(frame)
                self.orientations = np.vstack(
                    (self.orientations, np.array(orientation))
                )
                if self.debug:
                    if self.orientations.shape[0] % 100 == 0:
                        print(f"GNHPR - {self.orientations.shape}")
            except Exception as _:
                print(f"GNHPR#{self.orientations.shape[0]:04d}, Error")

        elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
            try:
                velocity = BESTNAV_solver(frame)
                self.velocities = np.vstack((self.velocities, np.array(velocity)))
                if self.debug:
                    if self.velocities.shape[0] % 100 == 0:
                        print(f"BESTNAVA - {self.velocities.shape}")
            except Exception as _:
                print(f"BESTNAVA#{self.velocities.shape[0]:04d}, Error")

    def run(self):
        self.running = True
        while self.running:
            self.read_frame()
            # print("Simulate reading data from GNSS device")
            # time.sleep(0.01)
            self.frame_no += 1

    def stop(self):
        self.running = False
        time.sleep(0.1)
        self.device.close()


def main():
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
    gnss_driver = UM982Driver(**args)
    gnss_driver.start()


if __name__ == "__main__":
    main()
