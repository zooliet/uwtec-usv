import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion

from geopy.distance import geodesic
from pyproj import Geod


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def euler_from_quaternion(q: Quaternion):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose


# def distance_and_bearing(lat1, lon1, lat2, lon2):
def distance_and_bearing(point1, point2):
    lat1 = point1[0]
    lon1 = point1[1]
    lat2 = point2[0]
    lon2 = point2[1]

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
        lat2_rad
    ) * math.cos(lon2_rad - lon1_rad)
    # print(x, y)

    theta_rad = math.atan2(y, x)
    bearing = (theta_rad * 180 / math.pi + 360) % 360

    point1 = (lat1, lon1)
    point2 = (lat2, lon2)
    # point1 = (48.1372, 11.5756)  # Munich (latitude, longitude)
    # point2 = (52.5186, 13.4083)  # Berlin (latitude, longitude)

    distance = geodesic(point1, point2).m
    # print(f"Distance: {distance:.2f} m")

    return distance, bearing


def calc_goal_heading(current_heading, by):
    goal_heading = (current_heading + by) % 360
    # print(f"Current: {current_heading}\tby {by}\tGoal: {goal_heading}")
    return goal_heading


def rotate_to_go(current_heading, goal_heading):
    deg = (goal_heading - current_heading) % 360
    if deg > 180:
        deg = deg % -360
    # if deg > 180:
    #     deg = deg - 360
    # elif deg < -180:
    #     deg = deg + 360
    # else:
    #     pass
    return deg


def coordinate_after_move(lat, lon, vel_east, vel_north, interval):
    geod = Geod(ellps="WGS84")
    dist_east = vel_east * interval
    dist_north = vel_north * interval
    distance = math.sqrt(math.pow(dist_east, 2) + math.pow(dist_north, 2))
    azimuth = math.degrees(math.atan2(dist_north, dist_east))
    lon, lat, _ = geod.fwd(lons=lon, lats=lat, az=azimuth, dist=distance)
    return (lat, lon)  # 순서 주의


if __name__ == "__main__":
    import argparse

    headings = [0, 10, 170, 270, 350]
    for heading in headings:
        calc_goal_heading(heading, 30)

    for heading in headings:
        calc_goal_heading(heading, -30)
    print("\n\n")
    ap = argparse.ArgumentParser()
    ap.add_argument("--heading", type=int, default=0, help="current heading reading")
    ap.add_argument("--by", type=int, default=30, help="degree to turn")
    args = vars(ap.parse_args())
    # print(args)
    current_heading = int(args.get("heading", 0))
    by = int(args.get("by", 30))

    goal = calc_goal_heading(current_heading, by)

    ranges = []
    start = current_heading
    end = current_heading + by
    if by > 0:
        if end < 360:
            ranges.append((start, end))
        else:
            ranges.append((start, 360))
            ranges.append((0, end % 360))
    elif by < 0:
        if end >= 0:
            ranges.append((start, end))
        else:
            ranges.append((start, 0))
            ranges.append((360, end % 360))
    else:
        pass

    print(ranges)

    for r in ranges:
        (start, end) = r
        inc = 1 if end >= start else -1
        for c in range(start, end, inc):
            deg = rotate_to_go(c, goal)
            print(f"{c}\tto\t{goal}\t=\t{deg}")
