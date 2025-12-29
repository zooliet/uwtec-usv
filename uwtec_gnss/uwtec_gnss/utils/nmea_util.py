import math
from pyproj import CRS, Transformer

def crc_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8, 0, -1):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table


NMEA_EXPEND_CRC_TABLE = crc_table()


def nmea_expend_crc(nmea_expend_sentence):
    def calculate_crc32(data):
        crc = 0
        for byte in data:
            crc = NMEA_EXPEND_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        sentence, crc = nmea_expend_sentence[1:].split("*")
        crc = crc[:8]
    except:
        return False
    calculated_crc = calculate_crc32(sentence.encode())
    return crc.lower() == format(calculated_crc, "08x")


def nmea_crc(nmea_sentence):
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2]
    except:
        return False
    calculated_checksum = 0
    for char in sentence:
        calculated_checksum ^= ord(char)
    calculated_checksum_hex = format(calculated_checksum, "X")
    return calculated_checksum_hex.zfill(2) == crc.upper()


def msg_seperate(msg: str):
    return msg[1 : msg.find("*")].split(",")


def PVTSLN_solver(msg: str):
    parts = msg_seperate(msg)
    bestpos_hgt = float(parts[10])
    bestpos_lat = float(parts[11])
    bestpos_lon = float(parts[12])
    bestpos_hgtstd = float(parts[13])
    bestpos_latstd = float(parts[14])
    bestpos_lonstd = float(parts[15])
    fix = (
        bestpos_hgt,
        bestpos_lat,
        bestpos_lon,
        bestpos_hgtstd,
        bestpos_latstd,
        bestpos_lonstd,
    )
    return fix


def GNHPR_solver(msg: str):
    parts = msg_seperate(msg)
    heading = float(parts[3 - 1])
    pitch = float(parts[4 - 1])
    roll = float(parts[5 - 1])
    orientation = (heading, pitch, roll)
    return orientation


def KSXT_solver(msg: str):
    parts = msg_seperate(msg)
    lon = float(parts[2])
    lat = float(parts[3])
    heading = float(parts[5])
    # heading = float(parts[3 - 1])
    # pitch = float(parts[4 - 1])
    # roll = float(parts[5 - 1])
    # orientation = (heading, pitch, roll)
    return (lon, lat, heading)


def BESTNAV_solver(msg: str):
    parts = msg_seperate(msg)
    vel_hor_std = float(parts[-1])
    vel_ver_std = float(parts[-2])
    vel_ver = float(parts[-3])
    vel_heading = float(parts[-4])
    vel_hor = float(parts[-5])
    vel_north = vel_hor * math.cos(math.radians(vel_heading))
    vel_east = vel_hor * math.sin(math.radians(vel_heading))
    return (vel_east, vel_north, vel_ver, vel_hor_std, vel_hor_std, vel_ver_std)


def create_utm_trans(lat, lon):
    zone_number = int((lon + 180) / 6) + 1
    isnorth = lat >= 0
    wgs84_crs = CRS("epsg:4326")
    utm_crs_str = f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
    utm_crs = CRS(utm_crs_str)
    transformer = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
    return transformer


def utm_trans(transformer, lon, lat):
    utm_x, utm_y = transformer.transform(lon, lat)
    return (utm_x, utm_y)

