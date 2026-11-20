import os

from setuptools import setup

package_name = "vr_safe_perception"


def _sim_data_files():
    """把 jaka_s5_sim 资源装到 share/vr_safe_perception/jaka_s5_sim/。

    源码树里 jaka_model/robot_frames 直接找包目录下的 jaka_s5_sim/；安装后
    Python 包目录里没有资源，就靠这份 share 副本（见 paths 和 _sim_asset）。
    """
    root = os.path.join(package_name, "jaka_s5_sim")
    out = []
    for dirpath, _dirs, files in os.walk(root):
        files = [f for f in files if not f.endswith((".pyc", ".pyo"))]
        if not files:
            continue
        rel = os.path.relpath(dirpath, root)
        dest = os.path.join("share", package_name, "jaka_s5_sim",
                            "" if rel == "." else rel)
        out.append((dest, [os.path.join(dirpath, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".tools"],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ] + _sim_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zwt",
    maintainer_email="zwt@example.com",
    description="VR_safe 相机观察源：五档判定 → 四值 SafetyObservation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "observation_node = vr_safe_perception.observation_node:main",
            "calibrate_zone = vr_safe_perception.tools.calibrate_zone:main",
            "calibrate_extrinsic = vr_safe_perception.tools.calibrate_extrinsic:main",
            "preview_zone = vr_safe_perception.tools.preview_zone:main",
            "record_clip = vr_safe_perception.tools.record_clip:main",
            "replay_clip = vr_safe_perception.tools.replay_clip:main",
        ],
    },
)
