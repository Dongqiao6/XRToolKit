from setuptools import find_packages, setup

package_name = "vr_safe_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zwt",
    maintainer_email="zwt@example.com",
    description="VR_safe Quest 遥操作源：目标、按键、心跳与系统软件急停",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_bridge = vr_safe_teleop.bridge_node:main",
        ],
    },
)
