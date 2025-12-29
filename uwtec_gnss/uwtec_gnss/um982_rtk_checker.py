import argparse
import serial
import time
import pynmea2


def parse_gps_data(device):
    while True:
        # Read a line from the serial port
        try:
            line = device.readline().decode("utf-8").strip()
        except Exception as e:
            print(e)
        finally:
            # print(line)
            pass

        # Check if the line is a complete NMEA sentence and contains 'GGA' data
        if line.startswith("$") and "GGA" in line:
            try:
                msg = pynmea2.parse(line)
                # print("\033[2J")
                print("\033[H\033[2J")
                print("-" * 20)
                print(f"Timestamp: {msg.timestamp}")
                print(f"Lat: {msg.lat}, Lon: {msg.lon}")
                print(f"Sats: {msg.num_sats}")
                print(f"GPS Quality: {msg.gps_qual}")
            except pynmea2.ChecksumError:
                print("Checksum mismatch, ignoring sentence.")
            except Exception as e:
                print(f"Error parsing sentence: {e}")
            finally:
                pass


# import requests
# from requests.auth import HTTPBasicAuth
#
# # NTRIP Server details
# # url = "https://ntrip.data.gnss.ga.gov.au:443/<YOUR LOCATION>"
# # url = "ntrip://gnss.eseoul.go.kr:2101"
# url = "ntrip://ntrip.rnav.info.hiroshima-cu.ac.jp/OEM7"
# username = "seoul"
# password = "seoul"
#
#
# def stream_ntrip():
#     headers = {
#         "Ntrip-Version": "Ntrip/2.0",
#         "User-Agent": "NTRIP PythonClient/1.0",
#         "Connection": "close",
#     }
#     try:
#         # Connect to NTRIP caster
#         response = requests.get(
#             url, headers=headers, stream=True, auth=HTTPBasicAuth(username, password)
#         )
#         print("Connected to NTRIP Caster.")
#         # Stream the corrections
#         for chunk in response.iter_content(chunk_size=1024):
#             # Write the RTCM data directly to the GNSS receiver
#             # ser.write(chunk)
#             print("Sending RTCM data to GNSS receiver...", chunk)
#     except requests.exceptions.RequestException as e:
#         print(f"Error connecting to NTRIP caster: {e}")
#     finally:
#         ser.close()
#         # print("Serial port closed.")
#


def configure_um982(port, baudrate):
    device = serial.Serial(port, baudrate)
    # suppress output
    cmd_str = "unlog\x0d\x0a".encode("utf-8")
    device.write(cmd_str)
    time.sleep(1)

    # enable gpgga
    cmd_str = "gpgga 1\x0d\x0a".encode("utf-8")
    device.write(cmd_str)
    time.sleep(1)

    cmd_str = "config sbas enable msas\x0d\x0a".encode("utf-8")
    device.write(cmd_str)
    time.sleep(1)

    cmd_str = "saveconfig\x0d\x0a".encode("utf-8")
    device.write(cmd_str)
    time.sleep(1)

    return device


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="/dev/ttyGNSS", help="GNSS device path")
    ap.add_argument("-b", "--baudrate", type=int, default=115200, help="baudrate")

    args = vars(ap.parse_args())
    print(args)

    device = configure_um982(args["port"], args["baudrate"])
    parse_gps_data(device)


if __name__ == "__main__":
    main()
