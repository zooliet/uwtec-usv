#! /usr/bin/env python

import argparse
import threading
import serial
import time


class MPU6050Driver(threading.Thread):
    def __init__(self, port, baudrate, debug):
        super().__init__()
        self.port = port
        self.port = baudrate
        self.debug = debug

        self.running = False
        self.heading = 0
        self.acc_x = 0
        self.acc_x = 0
        self.frame_no = 0

        self.device = serial.Serial(port, baudrate)

    def read_frame(self):
        try:
            frame = self.device.read(44)  # 11B*3 + 11B
            # frame = self.device.read(55)  # 11B*4 + 11B
            # found = 0
            rpy = []
            # accs = []
            for i in range(44 - 10):  # i: 0 - 33
                # for i in range(55 - 10):  # i: 0 - 44
                if frame[i] == 85 and frame[i + 1] == 83:
                    rpy = frame[i : i + 11]
                    check_sum = sum(rpy[:-1]) % 256
                    if check_sum == rpy[-1]:
                        self.heading = (rpy[6] + rpy[7] * 256) / 32768 * 180
                        # change heading direction to CW
                        self.heading = -(self.heading % -360)
                        # found += 1
                # elif frame[i] == 85 and frame[i + 1] == 81:
                #     accs = frame[i : i + 11]
                #     check_sum = sum(accs[:-1]) % 256
                #     if check_sum == accs[-1]:
                #         self.acc_x = (accs[2] + accs[3] * 256) / 16368  # * 156.8
                #         self.acc_y = (accs[4] + accs[5] * 256) / 16368  # * 156.8

                # if found == 2:
                if self.debug:
                    print(f"Frame No: {self.frame_no}\n{list(rpy)}\n{self.heading:.2f}")
                    # print(
                    #     f"Frame No: {self.frame_no}\n{list(rpy)}\n{list(accs)}\n{self.heading:.0f}, {self.acc_x:.0f}, {self.acc_y:.0f}"
                    # )
                break

        except Exception as e:
            print(e)

    def run(self):
        self.running = True
        while self.running:
            self.read_frame()
            time.sleep(0.01)
            self.frame_no += 1

    def stop(self):
        self.running = False
        time.sleep(0.1)
        self.device.close()


def main():
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
    gyro_driver = MPU6050Driver(**args)
    gyro_driver.start()


if __name__ == "__main__":
    main()
