#!/usr/bin/env python3
"""启动前预检：配置、模型、标定、相机、机器人/AMR、PC-service。失败就退出。"""

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def need(path, label):
    if not os.path.exists(path):
        print(f"✗ {label} 不存在：{path}", file=sys.stderr)
        return False
    return True


def load_yaml(path):
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except Exception as exc:
        print(f"✗ 读不了 {path}：{exc}", file=sys.stderr)
        return {}


def tcp_ok(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def hardware_checks():
    ok = True
    chassis = load_yaml(os.path.join(ROOT, "config", "chassis.yaml"))
    amr_ip = str((chassis.get("chassis") or {}).get("connect_ip", "192.168.71.50"))
    for label, host, port in (
        ("左臂 JAKA", "192.168.71.37", 10000),
        ("右臂 JAKA", "192.168.71.36", 10000),
        ("AMR", amr_ip, 5001),
    ):
        if tcp_ok(host, port):
            print(f"✓ {label} 可达 {host}:{port}")
        else:
            print(f"✗ {label} 不可达 {host}:{port}", file=sys.stderr)
            ok = False

    if shutil.which("pgrep") and subprocess.run(
            ["pgrep", "-f", "RoboticsServiceProcess"],
            stdout=subprocess.DEVNULL).returncode == 0:
        print("✓ PC-service 进程在跑")
    else:
        print("✗ PC-service（RoboticsServiceProcess）没在跑", file=sys.stderr)
        ok = False

    if shutil.which("lsusb"):
        out = subprocess.run(["lsusb"], capture_output=True, text=True).stdout.lower()
        if "2bc5" in out or "8086" in out:
            print("✓ 相机 USB 在线（Orbbec/RealSense）")
        else:
            print("✗ lsusb 没看到 Orbbec/RealSense 相机", file=sys.stderr)
            ok = False
    else:
        print("⚠ 没有 lsusb，跳过相机 USB 检查")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="dry_run", choices=["dry_run", "real"])
    ap.add_argument("--allow-uncalibrated", action="store_true")
    ap.add_argument("--skip-hardware", action="store_true",
                    help="只检查文件/配置，不查相机/IP/PC-service")
    args = ap.parse_args()
    ok = True
    ok &= need(os.path.join(ROOT, "config", "vr_safe.yaml"), "安全配置")
    if args.mode == "real":
        for rel in ("config/perception.yaml", "config/teleop.yaml", "config/chassis.yaml"):
            ok &= need(os.path.join(ROOT, rel), rel)
        for rel in ("models/pose_landmarker_full.task", "models/hand_landmarker.task"):
            ok &= need(os.path.join(ROOT, rel), rel)
        calib = os.path.join(ROOT, "config", "calibration", "extrinsic_calib.json")
        if need(calib, "外参标定"):
            try:
                with open(calib) as f:
                    d = json.load(f)
            except Exception as exc:
                print(f"✗ 外参标定读不了：{exc}", file=sys.stderr)
                ok = False
            else:
                if not d.get("ok", False) and not args.allow_uncalibrated:
                    print("✗ 外参标定 ok=false；生产拒绝启动。"
                          "开发可加 --allow-uncalibrated", file=sys.stderr)
                    ok = False
        ok &= need(os.path.join(ROOT, "config", "calibration", "zone_ground.json"),
                   "区域标定")
        if not args.skip_hardware:
            ok &= hardware_checks()
    if not ok:
        return 1
    print(f"✓ preflight 通过（{args.mode}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
