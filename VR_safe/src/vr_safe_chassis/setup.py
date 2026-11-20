from setuptools import find_packages, setup

package_name = "vr_safe_chassis"

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
    description="VR_safe AMR 底盘执行端：遥控、许可门、零速 keepalive、状态/里程计",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chassis_node = vr_safe_chassis.main:main",
        ],
    },
)
