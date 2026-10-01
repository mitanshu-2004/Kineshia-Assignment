import os
from glob import glob

from setuptools import setup

package_name = "planar_arm_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"),
            glob("models/*")),
        (os.path.join("share", package_name, "config"),
            glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mitanshu Goel",
    maintainer_email="mitanshug2004@gmail.com",
    description="Gazebo Sim (ros_gz) backend for the planar arm controller.",
    license="Proprietary — for evaluation use only",
    entry_points={
        "console_scripts": [
            "gazebo_controller_node = planar_arm_gazebo.gazebo_controller_node:main",
        ],
    },
)
