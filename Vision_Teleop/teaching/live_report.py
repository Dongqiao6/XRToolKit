"""Compact run evidence; presentation resampling never counts as observations."""
import json
import cv2
import numpy as np


def summarize(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = [json.loads(line) for line in (path/"metadata.jsonl").read_text().splitlines()]
    robot = [r for r in rows if r["type"] == "robot"]
    report = json.loads((path/"summary.json").read_text())
    if not robot:
        return
    t = np.array([r["time_s"] for r in robot])
    fig,axes = plt.subplots(4,1,figsize=(11,10),sharex=True,layout="constrained")
    origin = np.asarray(robot[0]["commanded_position"])
    for i,name in enumerate("xyz"):
        axes[0].plot(t,[(r.get("mapped_human_position",r["desired_position"])[i]-origin[i])*1000
                       if r["initialized"] and r["position_used"] else np.nan for r in robot],"--",label=f"mapped human {name}")
        axes[0].plot(t,[(r["actual_position"][i]-origin[i])*1000 for r in robot],label=f"actual {name}")
    axes[0].set_ylabel("Relative position (mm)")
    axes[0].legend(ncol=3,fontsize=8)
    axes[1].plot(t,[r["position_error_m"]*1000 for r in robot],label="command error (mm)")
    axes[1].plot(t,[r["orientation_error_deg"] for r in robot],label="command error (deg)")
    axes[1].legend()
    axes[2].plot(t,[r["source_age_ms"] if r["source_age_ms"] is not None else np.nan for r in robot])
    axes[2].set_ylabel("Source age (ms)")
    states = list(dict.fromkeys(r["state"] for r in robot))
    axes[3].step(t,[states.index(r["state"]) for r in robot],where="post")
    axes[3].set_yticks(range(len(states)),states,fontsize=8)
    axes[3].set_xlabel("Wall time from input start (s)")
    fig.suptitle("Recorded-input dry run" if report["dry_run"] else "Live right-hand run")
    fig.savefig(path/"tracking.png",dpi=140)
    plt.close(fig)
    preview = sorted([r for r in rows if r["type"] == "video_frame" and r["role"] == "preview"],key=lambda r:r["video_index"])
    repeats = 0
    if preview:
        times = np.array([r["host_arrival_monotonic_ns"] for r in preview],float)/1e9
        times -= times[0]
        cap = cv2.VideoCapture(str(path/"preview.avi"))
        writer = cv2.VideoWriter(str(path/"preview_timed.mp4"),cv2.VideoWriter_fourcc(*"mp4v"),20,(1280,480))
        try:
            if not writer.isOpened():
                raise RuntimeError("Cannot encode timestamp-paced preview")
            index,previous = -1,-1
            for stamp in np.arange(0,times[-1]+.025,.05):
                selected = max(0,int(np.searchsorted(times,stamp,side="right")-1))
                while index < selected:
                    ok,image = cap.read()
                    if not ok: raise RuntimeError("Missing preview frame")
                    index += 1
                repeats += int(selected == previous)
                writer.write(image)
                previous = selected
        finally:
            cap.release()
            writer.release()
    ready = next((r["time_s"] for r in robot if r["initialized"]),None)
    recovery = [r["orientation_error_deg"] for r in robot if r["state"] == "ORIENTATION_RECOVERING"]
    report.update(initialized_at_s=ready,max_recovery_orientation_error_deg=max(recovery,default=None),
                  render_hz=len(preview)/max(report["elapsed_s"],.001),
                  timed_video_repeated_presentations=repeats)
    (path/"summary.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    (path/"report.md").write_text(
        f"# 右手链路检查\n\n输入：{'已有录像按原时序播放，无相机启动' if report['dry_run'] else '实时相机'}。\n\n"
        f"- 双路识别完成帧对输出间隔频率：{report['observation_interval_hz']:.2f} Hz；包含几何不合格的观测，不是有效控制或控制循环频率。\n"
        f"- 处理帧龄 p95：{report['age_ms']['p95']:.1f} ms；不含人体真实运动到曝光的时间。\n"
        f"- 初始化完成时间：{ready} s；未初始化期间没有人体目标控制。\n"
        f"- 最大命令跟踪误差：{report['max_position_error_m']*1000:.1f} mm / {report['max_orientation_error_deg']:.2f}°。\n"
        f"- 恢复段最大姿态命令误差：{max(recovery) if recovery else None}°。\n"
        f"- 写盘丢帧：{report['recorder_drops']}；运行错误：{report['error']}。\n\n"
        "preview_timed.mp4 按保存的显示时间补齐展示帧，重复展示不计为识别帧；preview.avi 是原始渲染序列，不代表真实播放速度。\n"
        "tracking.png 区分期望目标和实际执行，误差曲线仅针对限速后的命令。所有源索引、关键点、几何、状态在 metadata.jsonl。\n"
        "射线间距只检查几何一致性，不能证明厘米级精度；新旧检测器和门控不同，不能把跟踪时长变化全部归功于提速。\n"
        "结论：这是软件整链证据；真人跟手性、当前相机视角和真实采集吞吐仍需一次现场体验，不标注通过。\n",
        encoding="utf-8")
