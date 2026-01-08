from setuptools import find_packages, setup
import os
from glob import glob

package_name = "uwtec_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("config/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Junhyun Shin",
    maintainer_email="hl1sqi@gmail.com",
    description="TODO: Package description",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "01_gps_custom_topic = uwtec_nav.custom_gps_topic:main",
            "02_gps_fix_and_utmpos_topic = uwtec_nav.gps_fix_and_utmpos_topic:main",
            "03_go_dancing = uwtec_nav.go_dancing:main",
            "04_turn_around = uwtec_nav.turn_around:main",
            "05_distance_and_bearing = uwtec_nav.distance_and_bearing:main",
            # "06_collect_coords = uwtec_nav.collect_coords:main",
            "07_shuttle_run = uwtec_nav.shuttle_run:main",
            # "08_nav_to_waypoints = uwtec_nav.nav_to_waypoints:main",
        ],
    },
)
