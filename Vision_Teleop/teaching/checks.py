"""Offline component evidence. Never presented as human teaching validation."""
from datetime import datetime
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import time
import urllib.request

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np
import yaml

from .observation import Detector
from .robot import SimArm

DEMO_COMMIT = "03a1693e59e4f7cd84582c0fb30459b3bf18ad42"
MODEL_HASHES = {
    "ceb11c07298f95c50d7c5abeb906d03340c85f23aa79e3e66966e7fb6c307250",
    "68db326c5c6f3273456f803eae0faf5440478369f2c3ee67258d2f36512e8d76",
}
HAND_MODEL_HASHES = {
    "568d3ea97a5b142488366b67e036b6a5cb0a1fef9087a710cb8e66b6979fbac2",
    "39e858936bca0f94c09847d4e70b68a51d6c0adac61f36b457fcadb54621cd29",
}


def prepare(root, config, skip_detector=False):
    output = root / "results/preparation" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    report = {"scope": "offline component checks, not end-to-end teaching", "cameras_started": False,
              "versions": {p: version(p) for p in ("rtmlib", "opencv-contrib-python", "onnxruntime-gpu", "pyrealsense2", "mink", "mujoco")},
              "official_scenes": [], "robots": [], "detector": None, "failure": None}
    try:
        settings = yaml.safe_load((root / "config/robots.yaml").read_text())
        for name, spec in settings.items():
            model = mujoco.MjModel.from_xml_path(str(root / spec["scene"]))
            data = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
            initial = data.qpos.copy()
            for _ in range(1000):
                mujoco.mj_step(model, data)
            drift = float(np.max(np.abs(data.qpos - initial)))
            before = data.qpos.copy()
            data.ctrl[0] += .01
            for _ in range(500):
                mujoco.mj_step(model, data)
            response = float(data.qpos[0] - before[0])
            if not np.isfinite(data.qpos).all() or drift > .2 or response < .001:
                raise RuntimeError(f"Unmodified official scene failed: {name}, drift={drift}, response={response}")
            report["official_scenes"].append(dict(name=name, unchanged=True, dof=model.nq,
                                                 home_drift_rad=drift, joint_command_rad=.01, response_rad=response))
        for name in ("franka_fr3", "ur5e"):
            for angle in (0, 45, 90):
                arm = SimArm(name, angle)
                position, orientation = arm.pose()
                target = position + np.array([0.01, 0, 0])
                arm.set_target(target, orientation)
                first = arm.data.qpos.copy()
                max_velocity = 0.0
                started = time.perf_counter()
                for _ in range(150):
                    v = arm.step()
                    max_velocity = max(max_velocity, float(np.max(np.abs(v))))
                actual, _ = arm.pose()
                cam = mujoco.MjvCamera()
                mujoco.mjv_defaultFreeCamera(arm.model, cam)
                cam.lookat[:] = [0, 0, 1.2]
                cam.distance, cam.azimuth, cam.elevation = 2.5, 130, -20
                with mujoco.Renderer(arm.model, height=480, width=640) as renderer:
                    renderer.update_scene(arm.data, camera=cam)
                    frame = renderer.render().copy()
                imageio.imwrite(output / f"{name}_{angle}.png", frame)
                row = {"name": name, "mount_deg": angle, "dof": arm.dof, "actuators": arm.model.nu,
                       "joint_motion_norm": float(np.linalg.norm(arm.data.qpos - first)),
                       "target_error_m": float(np.linalg.norm(target - actual)),
                       "max_command_joint_velocity": max_velocity, "image_std": float(frame.std()),
                       "wall_s": time.perf_counter() - started}
                report["robots"].append(row)
                if row["image_std"] < 5 or row["target_error_m"] > 0.05 or max_velocity > 0.601:
                    raise RuntimeError(f"Robot component check failed: {row}")
                print(f"Robot component {name} mount {angle}: {row['target_error_m']:.4f}m", flush=True)
        if not skip_detector:
            hand = config["detector"].get("target") == "hand"
            sample = root / ("models/hand_demo.jpg" if hand else "models/rtmlib_demo.jpg")
            sample.parent.mkdir(parents=True, exist_ok=True)
            source = f"https://raw.githubusercontent.com/Tau-J/rtmlib/{DEMO_COMMIT}/demo.jpg"
            expected_sample_hash = "43eb2c025e98b0ca2db567b6eac31551063f2cfcdac321330e5fc12bc8819031"
            if hand:
                source = "https://raw.githubusercontent.com/open-mmlab/mmpose/5408bc76f5b848cf925a0d1857899011d8c5b497/tests/data/onehand10k/9.jpg"
                expected_sample_hash = "3943863e90a6e31b6e9398ce5c6fb319069f5243aa7758f0deab97a9d760cf5b"
            if not sample.exists():
                with urllib.request.urlopen(source, timeout=60) as response:
                    sample.write_bytes(response.read())
            if hashlib.sha256(sample.read_bytes()).hexdigest() != expected_sample_hash:
                raise RuntimeError("Pinned upstream demo checksum mismatch")
            image = cv2.imread(str(sample))
            if image is None:
                raise ValueError("Upstream demo image cannot be decoded")
            detector = Detector(**config["detector"])
            files = {}
            for model in (detector.model.det_model, detector.model.pose_model):
                path = Path(model.onnx_model)
                files[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
            if set(files.values()) != (HAND_MODEL_HASHES if detector.target == "hand" else MODEL_HASHES):
                raise RuntimeError("Unverified detector weights; rerun source review before changing the selected model")
            for _ in range(3):
                keypoints, scores = detector(image)
            measurements = []
            for _ in range(15):
                started = time.perf_counter()
                keypoints, scores = detector(image)
                detector(image)
                measurements.append((time.perf_counter() - started) * 1000)
            from rtmlib import draw_skeleton
            annotated = draw_skeleton(image.copy(), keypoints, scores, kpt_thr=0.5)
            cv2.imwrite(str(output / "upstream_detection.jpg"), annotated)
            report["detector"] = {"providers": detector.providers, "shape": list(keypoints.shape),
                                  "single_subject_path": config["detector"].get("single_subject", False),
                                  "detector_target": detector.target, "detected_count": detector.person_count,
                                  "scores_above_half": int((scores >= 0.5).sum()),
                                  "serial_two_inference_mean_ms": float(np.mean(measurements)),
                                  "serial_two_inference_p95_ms": float(np.percentile(measurements, 95)),
                                  "benchmark_pair_hz": float(1000 / np.mean(measurements)),
                                  "sample_source": source, "sample_sha256": hashlib.sha256(sample.read_bytes()).hexdigest(),
                                  "model_sha256": files,
                                  "input_scope": "Upstream still repeated; not live FPS, handedness verification or palm accuracy validation"}
            if keypoints.ndim != 3 or keypoints.shape[1:] != (detector.keypoint_count, 2) or not len(keypoints):
                raise RuntimeError(f"Unexpected wholebody output: {keypoints.shape}")
        report["status"] = "ROBOT_ONLY_PARTIAL" if skip_detector else "COMPONENTS_RUN_HUMAN_INPUT_UNVERIFIED"
    except BaseException as exc:
        report["failure"] = f"{type(exc).__name__}: {exc}"
        report["status"] = "FAILED"
        raise
    finally:
        (output / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Preparation evidence: {output}", flush=True)
