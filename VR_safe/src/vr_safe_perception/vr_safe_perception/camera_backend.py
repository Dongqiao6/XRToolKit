#!/usr/bin/env python3
"""相机抽象层：把 RealSense 和 Orbbec 包成同一个接口。

换相机这件事不该波及判定逻辑。区域占用、手势、坐标变换这些和相机型号
没有半点关系，只有「怎么拿到一帧对齐好的彩色+深度+内参」是相机相关的。
所以把那一小块隔出来，上层只认这里定义的 Frame 和 Intrinsics。

统一约定（和之前 D435i 那套完全一致，已经 5/5 交叉验证过）：

    坐标系   +X 右 / +Y 下 / +Z 前，原点在**彩色**相机光心
    深度     米，0 表示没测到（空洞），不是 0 米
    对齐     深度一律对齐到彩色，两者共用同一套内参和同一个光心
    去畸变   反投影前自动做。D435i 固件已去过（coeffs 全零，这步是空操作），
             Orbbec 335L 没去过（k1=-0.0297），不补会在画面边缘偏几厘米

用法：

    cam = open_camera()            # 自动挑在线的那台
    cam = open_camera("orbbec")    # 或者指死
    with cam:
        for f in cam.frames():
            f.color      (H,W,3) BGR uint8
            f.depth      (H,W)   float32，米
            f.intr       Intrinsics
"""
import math
import os
import time
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Intrinsics:
    """针孔模型的内参。两家 SDK 字段名不同，这里统一。"""
    fx: float
    fy: float
    ppx: float
    ppy: float
    width: int
    height: int
    coeffs: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)

    def __post_init__(self):
        self._distorted = any(abs(c) > 1e-6 for c in self.coeffs)
        self._K = np.array([[self.fx, 0, self.ppx],
                            [0, self.fy, self.ppy], [0, 0, 1]], float)
        self._D = np.array(self.coeffs, float)

    def undistort(self, pts_px):
        """把带畸变的像素坐标映射回理想针孔模型的像素坐标。

        D435i 的彩色流固件已经去过畸变（实测 coeffs 全零），这一步是空操作；
        **Orbbec Gemini 335L 没有**，实测 k1=-0.0297、k2=+0.0332。
        画面边缘按理想针孔反投影会偏：r≈1 处约差 3%，848 宽的图在边缘
        就是十几个像素，0.8 m 处折合两三厘米——判区域边界时足够把内外判反。

        pts_px: (N,2)。返回同形状。
        """
        pts = np.asarray(pts_px, np.float64).reshape(-1, 1, 2)
        if not self._distorted:
            return pts.reshape(-1, 2)
        # P 给回 K，让结果还是像素坐标而不是归一化坐标
        out = cv2.undistortPoints(pts, self._K, self._D, P=self._K)
        return out.reshape(-1, 2)

    def deproject(self, px, py, depth_m):
        """像素 + 深度 → 三维点（米）。针孔模型的反投影，**先去畸变**。

        自己实现而不是调 SDK：两家的函数名和参数顺序都不一样，而公式只有
        三行，统一在这里反而少一处分叉。
        """
        if self._distorted:
            px, py = self.undistort([[px, py]])[0]
        return np.array([(px - self.ppx) / self.fx * depth_m,
                         (py - self.ppy) / self.fy * depth_m,
                         depth_m], dtype=float)

    def deproject_many(self, pts_px, depths_m):
        """批量反投影。关键点动辄几十个，逐个调 cv2 开销白给。

        pts_px: (N,2)   depths_m: (N,)   返回 (N,3)
        """
        pts = self.undistort(pts_px)
        d = np.asarray(depths_m, float).reshape(-1)
        return np.stack([(pts[:, 0] - self.ppx) / self.fx * d,
                         (pts[:, 1] - self.ppy) / self.fy * d, d], axis=1)

    def project(self, p3):
        """三维点 → 像素。点在相机背后返回 None。"""
        if p3[2] <= 1e-6:
            return None
        return (p3[0] / p3[2] * self.fx + self.ppx,
                p3[1] / p3[2] * self.fy + self.ppy)

    def fov_deg(self):
        return (2 * math.degrees(math.atan(self.width / 2 / self.fx)),
                2 * math.degrees(math.atan(self.height / 2 / self.fy)))


# 取帧时丢掉 SDK 队列里积压的旧帧、只要最新一帧。设 CAM_LATEST=0 恢复逐帧处理（离线录制要全帧时用）。
LATEST_ONLY = os.environ.get("CAM_LATEST", "1") == "1"


@dataclass
class Frame:
    color: np.ndarray        # (H,W,3) BGR uint8
    depth: np.ndarray        # (H,W) float32，米，0 = 没测到
    intr: Intrinsics
    t: float                 # 距开流的秒数
    t_sys: float = None      # SDK 收到这帧时的主机时间（time.time() 口径），算帧龄用；拿不到为 None


class _Base:
    name = "?"

    def read_gravity(self, secs=3.0):
        """静止采若干秒加速度计，返回重力在**彩色相机系**下的单位向量。

        加速度计测的是比力（支撑力），静止时方向朝上，所以重力是它的反号。
        IMU 和彩色相机不在同一个位置，中间差一个出厂标定的旋转；只比方向的话
        平移无所谓，但旋转必须转过去，否则算出来的俯仰角是错的。

        返回 (g_unit, 统计 dict)；相机没有 IMU 就返回 (None, 原因)。
        """
        raise NotImplementedError

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def frames(self):
        while True:
            f = self.read()
            if f is not None:
                yield f


class RealSense(_Base):
    name = "realsense"

    def __init__(self, width=848, height=480, fps=30):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        prof = self.pipe.start(cfg)
        self.prof = prof
        self.scale = prof.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = Intrinsics(i.fx, i.fy, i.ppx, i.ppy, i.width, i.height,
                               tuple(i.coeffs))
        d = prof.get_device()
        self.info = (f"{d.get_info(rs.camera_info.name)} "
                     f"{d.get_info(rs.camera_info.serial_number)}")
        self.t0 = time.time()

    def read(self):
        fs = self.align.process(self.pipe.wait_for_frames())
        d, c = fs.get_depth_frame(), fs.get_color_frame()
        if not d or not c:
            return None
        return Frame(np.asanyarray(c.get_data()),
                     np.asanyarray(d.get_data()).astype(np.float32) * self.scale,
                     self.intr, time.time() - self.t0)

    def close(self):
        try:
            self.pipe.stop()
        except Exception:
            pass


# Orbbec SDK 的 Context 不能重复建：第二次 query_devices 会枚举不到设备
# （第一个 Context 还占着 USB 句柄）。所以全局只留一个，谁要谁取。
_OB_CTX = None


def _ob_context():
    global _OB_CTX
    import pyorbbecsdk as ob
    if _OB_CTX is None:
        _OB_CTX = ob.Context()
    return _OB_CTX


class Orbbec(_Base):
    name = "orbbec"

    def __init__(self, width=848, height=480, fps=30):
        import pyorbbecsdk as ob
        self.ob = ob
        devs = _ob_context().query_devices()
        if devs.get_count() == 0:
            raise RuntimeError("Context 里一台 Orbbec 都没有")
        self.pipe = ob.Pipeline(devs.get_device_by_index(0))
        cfg = ob.Config()
        cp = self._pick(ob.OBSensorType.COLOR_SENSOR, width, height, fps,
                        ob.OBFormat.BGR if hasattr(ob.OBFormat, "BGR") else None)
        dp = self._pick(ob.OBSensorType.DEPTH_SENSOR, width, height, fps, None)
        cfg.enable_stream(cp)
        cfg.enable_stream(dp)
        # 深度对齐到彩色。两个物理成像器之间有基线，不对齐的话彩色图上的
        # (px,py) 在深度图里对应的是另一处，近距离误差能到几厘米。
        cfg.set_align_mode(ob.OBAlignMode.SW_MODE)
        self.pipe.start(cfg)
        p = self.pipe.get_camera_param()
        i = p.rgb_intrinsic
        d = p.rgb_distortion
        self.intr = Intrinsics(i.fx, i.fy, i.cx, i.cy, i.width, i.height,
                               (d.k1, d.k2, d.p1, d.p2, d.k3))
        try:
            di = self.pipe.get_device().get_device_info()
            self.info = f"{di.get_name()} {di.get_serial_number()}"
        except Exception:
            self.info = "Orbbec"
        self.scale = None          # 第一帧再读，各机型不同
        self.t0 = time.time()

    def _pick(self, sensor, w, h, fps, fmt):
        """挑一个最接近请求的流档。挑不到就退回默认档。"""
        pl = self.pipe.get_stream_profile_list(sensor)
        best, bestd = None, None
        for k in range(pl.get_count()):
            try:
                v = pl.get_stream_profile_by_index(k).as_video_stream_profile()
            except Exception:
                continue
            if v.get_fps() != fps:
                continue
            if fmt is not None and v.get_format() != fmt:
                continue
            d = abs(v.get_width() - w) + abs(v.get_height() - h)
            if bestd is None or d < bestd:
                best, bestd = v, d
        return best if best is not None else pl.get_default_video_stream_profile()

    def read(self):
        fs = self.pipe.wait_for_frames(200)
        if fs is None:
            return None
        # 只要最新的一帧。处理一帧比相机出一帧慢时，SDK 队列里会越积越多，
        # 拿到的都是几百毫秒前的旧画面（11-16 实测帧龄 350 ms）。把积压的倒掉。
        n_drop = 0
        while LATEST_ONLY and n_drop < 30:
            nxt = self.pipe.wait_for_frames(0)
            if nxt is None:
                break
            fs, n_drop = nxt, n_drop + 1
        c, d = fs.get_color_frame(), fs.get_depth_frame()
        if c is None or d is None:
            return None
        if self.scale is None:
            # Orbbec 的 depth_scale 是「原始值 → 毫米」，RealSense 是「→ 米」，
            # 这里统一成米，别在上层各算各的
            self.scale = d.get_depth_scale() / 1000.0
        import cv2
        ca = np.frombuffer(c.get_data(), dtype=np.uint8)
        h, w = c.get_height(), c.get_width()
        if ca.size == h * w * 3:
            color = ca.reshape(h, w, 3)
        else:
            color = cv2.imdecode(ca, cv2.IMREAD_COLOR)   # MJPG 档
        da = np.frombuffer(d.get_data(), dtype=np.uint16).reshape(
            d.get_height(), d.get_width())
        depth = da.astype(np.float32) * self.scale
        # 65535 是 uint16 的饱和值，表示「这个像素测不出来」，不是 65.535 米。
        # 不清掉的话，下游算中位数、拟合平面、判区域全会被这些假点带偏。
        # 顺手把超过 10 m 的也清掉——室内场景不可能，多半是多路径反射。
        depth[(da == 65535) | (depth > 10.0)] = 0.0
        t_sys = None
        try:   # 帧到达主机的时间，用来量「从相机给出这帧到判定出结果」的帧龄
            t_sys = c.get_system_timestamp_us() / 1e6
        except Exception:
            pass
        return Frame(np.ascontiguousarray(color), depth,
                     self.intr, time.time() - self.t0, t_sys)

    def read_gravity(self, secs=3.0):
        """Orbbec 的 IMU 要单独开一条 pipeline，和彩色/深度那条分开。

        SDK 把 accel 和 gyro 当成独立 sensor，不能塞进已经在跑的那条流里。
        """
        ob = self.ob
        try:
            devs = _ob_context().query_devices()
            dev = devs.get_device_by_index(0)
            ipipe = ob.Pipeline(dev)
            icfg = ob.Config()
            apl = ipipe.get_stream_profile_list(ob.OBSensorType.ACCEL_SENSOR)
            # 这个 SDK 的 StreamProfileList 没有 get_default_stream_profile，
            # 只有 get_stream_profile_by_index / get_accel_stream_profile。
            # 拿第 0 档就行，IMU 的档位只差采样率，方向数据是一样的。
            icfg.enable_stream(apl.get_stream_profile_by_index(0))
            ipipe.start(icfg)
        except Exception as e:
            return None, {"错误": f"IMU 开不了：{e}"}
        acc = []
        try:
            t0 = time.time()
            while time.time() - t0 < secs:
                fs = ipipe.wait_for_frames(200)
                if fs is None:
                    continue
                af = fs.get_frame(ob.OBFrameType.ACCEL_FRAME)
                if af is None:
                    continue
                a = af.as_accel_frame().get_value()
                acc.append([a.x, a.y, a.z])
        finally:
            try:
                ipipe.stop()
            except Exception:
                pass
        if not acc:
            return None, {"错误": "一条加速度数据都没读到"}
        A = np.array(acc)
        # Orbbec 的 IMU 坐标系和深度相机系对齐（官方口径），这里不另做旋转；
        # 对不对由「和桌面法向交叉验证」那一步兜底——两者差太多就说明要补外参。
        g = -A.mean(axis=0)
        return g / np.linalg.norm(g), {
            "样本": len(A), "原始均值": A.mean(axis=0).tolist(),
            "模长": float(np.linalg.norm(A.mean(axis=0))),
            "三轴σ": A.std(axis=0).tolist(),
            "注": "Orbbec IMU 按官方口径与深度相机系对齐，未另加外参旋转"}

    def close(self):
        try:
            self.pipe.stop()
        except Exception:
            pass


def available(verbose=False):
    """看哪些后端此刻能用。verbose=True 时把失败原因打出来。

    注意 Orbbec 这边 **Context 和 DeviceList 都要保持引用**：
    写成 `ob.Context().query_devices()` 的话，Context 是临时对象，
    查完立刻被 GC，返回的 DeviceList 就指向已释放的 deviceMgr，
    下一次访问报 "NULL pointer passed for argument deviceMgr"。
    """
    out = []
    try:
        ctx = _ob_context()
        devs = ctx.query_devices()      # 必须落到变量上，别链式调用
        n = devs.get_count()
        if verbose:
            print(f"  orbbec: 枚举到 {n} 台")
        if n > 0:
            out.append("orbbec")
    except Exception as e:
        if verbose:
            print(f"  orbbec 不可用: {type(e).__name__}: {e}")
    try:
        import pyrealsense2 as rs
        c = rs.context()
        ds = c.query_devices()
        if verbose:
            print(f"  realsense: 枚举到 {len(ds)} 台")
        if len(ds) > 0:
            out.append("realsense")
    except Exception as e:
        if verbose:
            print(f"  realsense 不可用: {type(e).__name__}: {e}")
    return out


def _term_as_interrupt():
    """让 SIGTERM 和 Ctrl+C 一样走清理流程。

    Python 收到 SIGTERM 默认是直接退出，try/finally 里的 cam.close() 不会跑，
    SDK 没机会关流，相机固件就卡在「能打开但不出帧」——11-14 在 Jetson 上
    踩过：用 kill 停了 rerun，下一个程序开相机 25 秒一帧不出，得 USB 复位才好。
    这里把 SIGTERM 换成 KeyboardInterrupt，各程序已有的 finally 就都能生效。
    只在主线程、且没人改过 SIGTERM 处理时装，不覆盖调用方自己的设置。
    """
    import signal
    import threading
    if threading.current_thread() is not threading.main_thread():
        return

    def _raise(signum, frame):
        raise KeyboardInterrupt
    if signal.getsignal(signal.SIGTERM) is signal.SIG_DFL:
        signal.signal(signal.SIGTERM, _raise)
    # 脚本里用 & 放到后台的程序，bash 会让它**忽略** SIGINT（POSIX 规矩），
    # Python 一看是忽略就不装 Ctrl+C 处理——于是 kill -INT 对它完全无效。
    # 11-14 踩过：stop.sh 发了 SIGINT，程序又继续跑了 15 秒。这里恢复回来。
    if signal.getsignal(signal.SIGINT) is signal.SIG_IGN:
        signal.signal(signal.SIGINT, signal.default_int_handler)


def open_camera(backend=None, width=848, height=480, fps=30):
    """开相机。backend 不给就自动挑在线的那台（Orbbec 优先）。"""
    _term_as_interrupt()
    cands = [backend] if backend else available()
    if not cands:
        raise RuntimeError("没有可用的相机。查 lsusb 和 udev 规则。")
    err = []
    for b in cands:
        try:
            cam = {"orbbec": Orbbec, "realsense": RealSense}[b](width, height, fps)
            print(f"相机 [{cam.name}] {cam.info}   "
                  f"{cam.intr.width}x{cam.intr.height}  "
                  f"视场 {cam.intr.fov_deg()[0]:.1f}°x{cam.intr.fov_deg()[1]:.1f}°")
            return cam
        except Exception as e:
            err.append(f"{b}: {e}")
    raise RuntimeError("都开不了 —— " + " | ".join(err))


if __name__ == "__main__":
    print("探测后端：")
    print("在线：", available(verbose=True) or "（无）")
    with open_camera() as cam:
        for i, f in enumerate(cam.frames()):
            if i >= 30:
                break
        v = f.depth[f.depth > 0]
        print(f"取到 {i+1} 帧   彩色 {f.color.shape}  深度 {f.depth.shape}")
        print(f"深度有效像素 {(f.depth>0).mean()*100:.1f}%   "
              f"中位 {np.median(v):.3f} m   范围 {v.min():.3f}~{v.max():.3f} m")
        print(f"内参 fx={f.intr.fx:.2f} fy={f.intr.fy:.2f} "
              f"ppx={f.intr.ppx:.2f} ppy={f.intr.ppy:.2f}")
        print(f"畸变 {f.intr.coeffs}")
