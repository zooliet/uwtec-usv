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
    # KSXT_solver,
)
import numpy as np


class UM982Driver(threading.Thread):
    def __init__(self, port, baudrate):
        super().__init__()
        self.device = serial.Serial(port, int(baudrate))
        self.running = False
        self.fixes = np.empty(shape=(0, 6))
        self.orientations = np.empty(shape=(0, 3))
        self.velocities = np.empty(shape=(0, 6))
        self.nmea = None

    def read_frame(self):
        try:
            frame = self.device.readline().decode("utf-8").strip()
        except Exception as e:
            print(e)
        finally:
            pass
        # For simplicity, we will just print the frame
        # print(f"Received frame: {frame.strip()}")
        if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
            try:
                fix = PVTSLN_solver(frame)
                self.fixes = np.vstack((self.fixes, np.array(fix)))
            except Exception as _:
                print(f"PVTSLNA#{self.fixes.shape[0]:04d}, Error")
            # if self.fixes.shape[0] % 100 == 0:
            #     print(f"PVTSLNA - {self.fixes.shape}")

        elif frame.startswith("$GNHPR") and nmea_crc(frame):
            try:
                orientation = GNHPR_solver(frame)
                self.orientations = np.vstack(
                    (self.orientations, np.array(orientation))
                )
            except Exception as _:
                print(f"GNHPR#{self.orientations.shape[0]:04d}, Error")
            # if self.orientations.shape[0] % 100 == 0:
            #     print(f"GNHPR - {self.orientations.shape}")

        elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
            try:
                velocity = BESTNAV_solver(frame)
                self.velocities = np.vstack((self.velocities, np.array(velocity)))
            except Exception as _:
                print(f"BESTNAVA#{self.velocities.shape[0]:04d}, Error")
            # if self.velocities.shape[0] % 100 == 0:
            #     print(f"BESTNAVA - {self.velocities.shape}")

    # elif frame.startswith("$KSXT") and nmea_crc(frame):
    #     self.ksxt = KSXT_solver(frame)

    def run(self):
        self.running = True
        while self.running:
            # print("Simulate reading data from GNSS device")
            # time.sleep(0.1)
            self.read_frame()

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

    args = vars(ap.parse_args())
    print(args)
    gnss_driver = UM982Driver(args["port"], args["baudrate"])
    gnss_driver.start()


if __name__ == "__main__":
    main()
