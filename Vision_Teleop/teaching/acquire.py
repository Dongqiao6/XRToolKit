"""One small native control window; OpenCV displays the original camera views."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import copy
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import numpy as np
import yaml

from .capture import DualCapture, common_color_profiles, enumerate_devices
from .observation import BoardStability, Detector, calibrate, detect_board, palm_observation_clear, validate_board
from .recording import Recorder


def saved_session_message(pair_count, min_pairs, video_frames):
    left, right = video_frames.get("left", 0), video_frames.get("right", 0)
    if left or right:
        text = f"原始视频已保存：左 {left} 帧 / 右 {right} 帧。"
        if pair_count == 0:
            return text + "本会话未采标定板，不代表录像失败；三维处理需另关联匹配标定。"
        return text + f"本会话标定照片 {pair_count} 对，至少 {min_pairs} 对才计算标定。"
    return f"会话已保存：标定照片 {pair_count} 对，至少 {min_pairs} 对才计算；本会话未录动作视频。"


def run(config, root, auto_preview=False, hand_preview=False, config_path=None, auto_record=False):
    config = copy.deepcopy(config)
    devices = enumerate_devices()
    if len(devices) < 2:
        raise RuntimeError(f"Two RealSense devices required, found {len(devices)}")
    window = tk.Tk()
    window.title("双视角采集准备")
    window.geometry("700x480")
    font = ("Microsoft YaHei UI", 11)
    status = tk.StringVar(value="先预览并确认物理左右；标定板参数为内部角点数量。")
    camera_names = [f"{d['name']} | {d['serial']} | USB {d['usb']}" for d in devices]
    serials = [d["serial"] for d in devices]
    left = ttk.Combobox(window, values=camera_names, state="readonly", width=70)
    right = ttk.Combobox(window, values=camera_names, state="readonly", width=70)
    for widget, role, fallback in ((left, "left", 0), (right, "right", 1)):
        saved = config["cameras"].get(f"{role}_serial")
        widget.current(serials.index(saved) if saved in serials else fallback)
    for title, widget in (("左侧相机", left), ("右侧相机", right)):
        tk.Label(window, text=title, font=font).pack(anchor="w", padx=16)
        widget.pack(padx=16)
    tk.Label(window, text="共同支持的彩色模式（宽 × 高 @ FPS，BGR8）", font=font).pack(anchor="w", padx=16)
    resolution = ttk.Combobox(window, state="readonly", width=35)
    resolution.pack(padx=16)
    modes = []

    def refresh_modes(*args):
        nonlocal modes
        modes = common_color_profiles(devices[left.current()], devices[right.current()])
        resolution["values"] = [f"{w} x {h} @ {fps}" for w, h, fps in modes]
        desired = tuple(config["cameras"][k] for k in ("width", "height", "fps"))
        if desired in modes:
            resolution.current(modes.index(desired))
        else:
            resolution.set("")
            status.set("配置中的模式不被两台设备共同支持，请明确选择一个模式；程序不会自动降档。")

    left.bind("<<ComboboxSelected>>", refresh_modes)
    right.bind("<<ComboboxSelected>>", refresh_modes)
    refresh_modes()
    board_row = tk.Frame(window)
    board_row.pack(pady=10)
    fields = {}
    for key, title, default in (("cols", "内部角点列", config["board"]["cols"]),
                                 ("rows", "内部角点行", config["board"]["rows"]),
                                 ("size_mm", "方格边长 mm", None if config["board"]["square_size_m"] is None else config["board"]["square_size_m"] * 1000)):
        tk.Label(board_row, text=title, font=font).pack(side="left")
        field = tk.Entry(board_row, width=6)
        if default is not None:
            field.insert(0, str(default))
        field.pack(side="left", padx=5)
        fields[key] = field
    controls = tk.Frame(window)
    controls.pack(pady=8)
    tk.Label(window, textvariable=status, font=font, wraplength=640, justify="left").pack(padx=16, pady=10)
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vision")
    capture = recorder = pending = latest = detector = None
    closing, automatic = False, False
    mode = "preview"
    processed = None
    stability = BoardStability(config["board"]["stable_frames"], config["board"]["max_motion_px_per_frame"])
    last_save = 0.0
    generation = 0
    pending_generation = -1
    accepted_corners = []
    preview_count, preview_started = 0, 0.0

    def board_config():
        config["board"].update(cols=int(fields["cols"].get()), rows=int(fields["rows"].get()),
                               square_size_m=float(fields["size_mm"].get()) / 1000)
        validate_board(config["board"])

    def stop_capture():
        nonlocal capture, recorder
        try:
            if capture:
                capture.stop()
        finally:
            capture = None
            for widget in (left, right, resolution):
                widget.configure(state="readonly")
            for field in fields.values():
                field.configure(state="normal")
            if recorder:
                recorder.close()
                recorder = None

    def start():
        nonlocal capture, recorder, processed, latest, automatic, mode, generation
        nonlocal preview_count, preview_started
        if capture:
            status.set("先结束当前会话，再更换相机；当前录制不会被覆盖。")
            return
        try:
            config["cameras"].update(left_serial=serials[left.current()], right_serial=serials[right.current()])
            if config["cameras"]["left_serial"] == config["cameras"]["right_serial"]:
                raise ValueError("左右不能选择同一设备")
            if resolution.current() < 0:
                raise ValueError("请选择两台设备共同支持的彩色模式")
            config["cameras"].update(zip(("width", "height", "fps"), modes[resolution.current()]))
            recorder = Recorder(root / config["output_dir"], config)
            recorder.event({"type": "devices", "devices": devices})
            capture = DualCapture(config["cameras"], recorder.event, on_frame=recorder.frame).start()
            for widget in (left, right, resolution):
                widget.configure(state="disabled")
            preview_count, preview_started = 0, time.monotonic()
            generation += 1
            accepted_corners.clear()
            stability.history.clear()
            latest = processed = None
            automatic, mode = False, "preview"
            status.set("预览已开始，尚未录制人体。检查左右位置；左右反了请结束后交换上方选择。")
        except Exception as exc:
            stop_capture()
            messagebox.showerror("启动失败", str(exc))

    def calibration_mode():
        nonlocal mode, automatic, processed
        try:
            board_config()
            if not capture:
                raise ValueError("请先开始预览")
            from .calibration_state import save_config
            config['calibration_review_required'] = True
            config['calibration_review_note'] = 'New board acquisition started; do not reuse an earlier setup without review'
            save_config(config_path or root/'config/teaching.yaml',config)
            mode, automatic, processed = "board", True, None
            for field in fields.values():
                field.configure(state="disabled")
            status.set("自动采集：让完整棋盘格同时出现在两幅图中，改变位置/倾角后停稳。")
        except Exception as exc:
            messagebox.showerror("需要检查参数", str(exc))

    def record_motion():
        nonlocal mode, automatic, processed
        if not capture:
            status.set("请先开始预览。")
            return
        mode, automatic, processed = "body", False, None
        recorder.record_video.set()
        recorder.event({"type": "motion_recording_started", "monotonic_ns": time.monotonic_ns()})
        status.set("正在录制原始左右画面：仅展示右手，保持手腕和整只手可见，缓慢移动并转掌。结束后点击结束保存。")

    def observe_hand():
        nonlocal mode, automatic, processed
        if not capture:
            status.set("请先开始预览。")
            return
        mode, automatic, processed = "body", False, None
        status.set("手部检测加载中。当前不录视频；只展示右手，另一只手放到画面外。")

    def finish():
        nonlocal automatic, mode, processed
        if not recorder:
            return
        automatic, mode, processed = False, "preview", None
        path = recorder.path
        local_recorder = recorder
        try:
            stop_capture()
            rows = local_recorder.saved_pairs
            if len(rows) >= config["board"]["min_pairs"]:
                corners = [(np.asarray(r["left_corners"], np.float32), np.asarray(r["right_corners"], np.float32)) for r in rows]
                size = (rows[0]["left"]["width"], rows[0]["left"]["height"])
                result = calibrate(corners, size, config["board"])
                result["camera_serials"] = [config["cameras"]["left_serial"], config["cameras"]["right_serial"]]
                (path / "calibration.yaml").write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
                from .calibration_state import bind_calibration
                bind_calibration(root, config_path or root/'config/teaching.yaml', config, path/'calibration.yaml',
                                 reviewed=False, reason='New acquisition calibration; geometry review pending')
                status.set(f"已保存并关联新标定：{len(rows)} 对，RMS={result['stereo_rms']:.3f}px。待几何检查；后续不再沿用旧标定。\n{path}")
            else:
                status.set(saved_session_message(len(rows), config['board']['min_pairs'],
                                               local_recorder.manifest.get("video_frames", {})) + f"\n{path}")
        except Exception as exc:
            messagebox.showerror("保存/标定检查", f"{exc}\n原始会话保留在 {path}")

    def manual_save():
        if processed and processed[0] == "board" and all(x is not None for x in processed[2]) and recorder:
            if time.monotonic_ns() / 1e6 - processed[1][0].time_ms > 1000:
                status.set("检测结果已过期，请等待新一对。")
                return
            recorder.save_pair(processed[1], processed[2], processed[3], False)

    def process(pair, requested_mode):
        nonlocal detector
        if requested_mode == "board":
            corners = (detect_board(pair[0].image, config["board"]), detect_board(pair[1].image, config["board"]))
            return "board", pair, corners
        if detector is None:
            detector = Detector(**config["detector"])
        left_result = detector(pair[0].image)
        left_count = detector.person_count
        right_result = detector(pair[1].image)
        return "body", pair, (left_result, right_result), (left_count, detector.person_count)

    def tick():
        nonlocal pending, processed, latest, last_save, pending_generation
        nonlocal preview_count
        if closing:
            return
        try:
            if capture:
                recorder.check()
                pair = capture.get_pair()
                if pair:
                    latest = pair
                    preview_count += 1
                if pending and pending.done():
                    result = pending.result()
                    pending = None
                    if result[0] == mode and pending_generation == generation:
                        if mode == "board":
                            stable, motion = stability.update(result[2], [result[1][i].metadata["frame_number"] for i in (0, 1)])
                            processed = (*result, motion)
                            found = [x is not None for x in result[2]]
                            status.set(f"角点：左{'有' if found[0] else '无'} / 右{'有' if found[1] else '无'}；{'已停稳' if stable else '请停稳或调整角度'}；已写入 {len(recorder.saved_pairs)} 对")
                            joined = np.concatenate(result[2]).reshape(-1, 2) if all(found) else None
                            novel = joined is not None and all(np.linalg.norm(joined - old, axis=1).mean() >= 15 for old in accepted_corners)
                            if automatic and stable and novel and time.monotonic() - last_save >= 2:
                                if recorder.save_pair(result[1], result[2], motion, True):
                                    last_save = time.monotonic()
                                    accepted_corners.append(joined.copy())
                        else:
                            processed = (*result, None)
                            clear = [palm_observation_clear(r, config["detector"].get("min_palm_score", .5)) for r in result[2]]
                            recorder.event({"type": "keypoints_2d", "pair_id": result[1][2]["pair_id"],
                                            "left": [a.tolist() for a in result[2][0]], "right": [a.tolist() for a in result[2][1]],
                                            "detector_target": config["detector"].get("target", "body"),
                                            "detected_counts": result[3], "single_subject_unambiguous": result[3] == (1, 1),
                                            "palm_observation_clear": clear,
                                            "declared_hand": "right", "handedness_verified": False})
                            activity = "原始视频录制中（不依赖检测是否成功）" if recorder.record_video.is_set() else "仅预览，不录视频"
                            noun = "手" if config["detector"].get("target") == "hand" else "人"
                            status.set(f"{activity}；检测到{noun}：左 {result[3][0]} / 右 {result[3][1]}。" +
                                       (f"掌部点：左{'清楚' if clear[0] else '不足'} / 右{'清楚' if clear[1] else '不足'}；只展示右手。"
                                        if result[3] == (1, 1) else "数量不明确，不能作为控制输入。"))
                        debug = [result[1][i].image.copy() for i in (0, 1)]
                        for i in (0, 1):
                            if result[0] == "board" and result[2][i] is not None:
                                cv2.drawChessboardCorners(debug[i], (config["board"]["cols"], config["board"]["rows"]), result[2][i], True)
                            elif result[0] == "body":
                                from rtmlib import draw_skeleton
                                debug[i] = draw_skeleton(debug[i], *result[2][i], kpt_thr=0.5)
                        combined = np.hstack(debug)
                        cv2.imshow("Latest completed detection - separate from live preview", cv2.resize(combined, (1000, round(combined.shape[0] * 1000 / combined.shape[1]))))
                if pair and not pending and mode != "preview":
                    pending = executor.submit(process, pair, mode)
                    pending_generation = generation
                if latest:
                    images = [latest[i].image.copy() for i in (0, 1)]
                    # Only overlay a result on the exact frame that was detected;
                    # stale corners/landmarks must not be painted over newer images.
                    if processed and processed[1][2]["pair_id"] == latest[2]["pair_id"]:
                        for i, image in enumerate(images):
                            if processed[0] == "board" and processed[2][i] is not None:
                                cv2.drawChessboardCorners(image, (config["board"]["cols"], config["board"]["rows"]), processed[2][i], True)
                            elif processed[0] == "body":
                                from rtmlib import draw_skeleton
                                images[i] = draw_skeleton(image, *processed[2][i], kpt_thr=0.5)
                    for i, image in enumerate(images):
                        text = f"{'LEFT' if i == 0 else 'RIGHT'} {latest[i].metadata['camera_serial']} | host delta {latest[2]['timestamp_diff_ms']:.1f}ms"
                        cv2.putText(image, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 60), 2)
                        rate = preview_count / max(time.monotonic() - preview_started, .001)
                        cv2.putText(image, f"{image.shape[1]}x{image.shape[0]} | received pairs {rate:.1f}/s", (15, 55),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 60), 2)
                    preview = np.hstack(images)
                    factor = min(1, 1500 / preview.shape[1])
                    cv2.imshow("Camera views - original pixels saved separately", cv2.resize(preview, None, fx=factor, fy=factor))
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        shutdown()
                        return
        except Exception as exc:
            status.set(f"已停止：{exc}")
            stop_capture()
        window.after(15, tick)

    def shutdown():
        nonlocal closing
        closing = True
        try:
            finish()
        finally:
            executor.shutdown(wait=True, cancel_futures=True)
            cv2.destroyAllWindows()
            window.destroy()

    for index, (title, action) in enumerate((("开始预览", start), ("手部预览", observe_hand), ("自动采标定板", calibration_mode),
                                            ("手动存一对", manual_save), ("开始录动作", record_motion), ("结束保存", finish))):
        tk.Button(controls, text=title, command=action, font=font).grid(row=index // 3, column=index % 3, padx=5, pady=3, sticky="ew")
    window.protocol("WM_DELETE_WINDOW", shutdown)
    if auto_record:
        def start_recording():
            start()
            if capture is not None:
                record_motion()
        window.after(200, start_recording)
    elif auto_preview:
        window.after(200, start)
    if hand_preview:
        window.after(500, observe_hand)
    tick()
    window.mainloop()
