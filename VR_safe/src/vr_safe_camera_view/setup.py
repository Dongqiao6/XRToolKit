from setuptools import setup

package_name = "vr_safe_camera_view"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zwt",
    maintainer_email="zwt@example.com",
    description="VR_safe camera view",
    license="Apache-2.0",
    entry_points={"console_scripts": ["camera_view = vr_safe_camera_view.camera_view:main"]},
)
