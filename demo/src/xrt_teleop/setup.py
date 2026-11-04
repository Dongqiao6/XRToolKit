import os
from glob import glob

from setuptools import find_packages, setup


package_name = "xrt_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kevin",
    maintainer_email="kevin@todo.todo",
    description="XRoboToolkit Quest 3 input bridge for JAKA dual-arm teleoperation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "xrt_bridge = xrt_teleop.bridge_node:main",
        ],
    },
)
