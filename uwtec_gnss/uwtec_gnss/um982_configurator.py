#! /usr/bin/env python

import argparse
import serial
import time
import re
import yaml
import os

from ament_index_python.packages import get_package_share_directory


class UM982_Configurator:
    def __init__(self, configs):
        super().__init__()
        self.port = configs.pop("port")
        self.baudrate = configs.pop("baudrate")
        self.device = serial.Serial(self.port, self.baudrate)
        self.configs = configs

    def configure(self):
        cmd = self.configs.get("cmd", None)
        if cmd:
            cmd_str = f"{cmd}\x0d\x0a".encode("utf-8")
            self.device.write(cmd_str)
            time.sleep(1)

            if cmd.lower().startswith("config") and re.findall(r"\d+", cmd):
                baudrate = int(re.findall(r"\d+", cmd)[0])
                # close current device
                self.device.close()
                time.sleep(1)
                # open device with new baudrate
                self.device = serial.Serial(self.port, baudrate)

            # save
            cmd_str = "SAVECONFIG\x0d\x0a".encode("utf-8")
            self.device.write(cmd_str)
            time.sleep(1)
            print(f"{cmd.upper()} Done\n")

        device_name = self.configs.get("yaml", None)
        if device_name:
            package_name = "uwtec_gnss"  # Replace with your package name
            file_name = "gnss.yaml"
            try:
                share_directory = get_package_share_directory(package_name)
                config_path = os.path.join(share_directory, "config", file_name)
                with open(config_path, "r") as file:
                    configs = yaml.safe_load(file)
                    configs = configs[device_name]
                    print(configs)
                    for cmd, val in configs.items():
                        print(cmd, val)
                        cmd_str = f"{cmd} {val}\x0d\x0a".encode("utf-8")
                        self.device.write(cmd_str)
                        time.sleep(1)

                    cmd_str = "SAVECONFIG\x0d\x0a".encode("utf-8")
                    self.device.write(cmd_str)
                    time.sleep(1)
                    print(f"YAML configuration Done\n")

            except Exception as e:
                print(f"Could not load YAML file: {e}")

    def change_speed(self, baudrate):
        cmd_str = f"CONFIG COM3 {baudrate}\x0d\x0a".encode("utf-8")
        self.device.write(cmd_str)
        time.sleep(1)

        # close device
        self.device.close()
        time.sleep(1)
        # open device with new baudrate
        self.device = serial.Serial(self.port, baudrate)
        # save
        cmd_str = "SAVECONFIG\x0d\x0a".encode("utf-8")
        self.device.write(cmd_str)
        print(f"CONFIG {baudrate} Done\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="/dev/ttyGNSS", help="GNSS device path")
    ap.add_argument(
        "-b", "--baudrate", type=int, required=True, help="current baudrate"
    )
    ap.add_argument("-c", "--cmd", help="configuration command")
    ap.add_argument("-y", "--yaml", help="device profile in config/gnss.yaml")

    args = vars(ap.parse_args())
    print(args)
    configurator = UM982_Configurator(args)
    configurator.configure()


if __name__ == "__main__":
    main()
