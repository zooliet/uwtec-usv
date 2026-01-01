#! /usr/bin/env python

import argparse
import threading
import serial
import time


class MPU6050Driver(threading.Thread):
    def __init__(self, port, baudrate, debug):
        super().__init__()
        self.device = serial.Serial(port, int(baudrate))
        self.running = False
        self.heading = 0
        self.frame_no = 0
        self.debug = debug

    def read_frame(self):
        try:
            frame = self.device.read(44)  # 11B*3 + 11B
            for i in range(44 - 10):  # i: 0 - 33
                if frame[i] == 85 and frame[i + 1] == 83:
                    rpy = frame[i : i + 11]
                    check_sum = sum(rpy[:-1]) % 256
                    if check_sum == rpy[-1]:
                        self.heading = (rpy[6] + rpy[7] * 256) / 32768 * 180
                        # change heading direction to CW
                        self.heading = -(self.heading % -360)

                    if self.debug:
                        print(list(rpy))
                        print(f"{self.frame_no}: {self.heading:.2f}")
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
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Gyro device path")
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
