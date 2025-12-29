from setuptools import find_packages, setup
import os
from glob import glob

package_name = "uwtec_gnss"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Add this line to install yaml files from the config directory
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
            "um982_config = uwtec_gnss.um982_configurator:main",
            "um982_driver = uwtec_gnss.um982_driver:main",
            "um982_publisher = uwtec_gnss.um982_publisher:main",
            "um982_rkt_chekcer = uwtec_gnss.um982_rkt_chekcer:main",
        ],
    },
)
