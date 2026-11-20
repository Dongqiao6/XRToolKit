"""GPU 姿态后端：YOLOX 人体检测 + RTMW 全身 133 点，onnxruntime（TensorRT / CUDA EP）推理。

用途：替代 MediaPipe 的 PoseLandmarker + HandLandmarker（两者在 Jetson 上只能跑 CPU，
GPU delegate 报 `Failed to create custom tensor buffer.`）。输出和 MediaPipe 结果同形状，
下游判定代码不用改：

  res = det.detect_for_video(mp_image_or_rgb_ndarray, ts_ms)
  res.pose_landmarks : 每人一个长度 33 的列表，元素有 .x .y（0~1 归一化）.z(=0) .visibility .presence
  res.hand_landmarks : 每只手一个长度 21 的列表，元素有 .x .y .z(=0)，顺序同 MediaPipe

make_gpu_pose() / make_gpu_hands() 返回的两个对象共用同一个模型：同一帧（同一 ts_ms）
只推理一次，谁先调谁算，另一个直接取缓存——所以 zone_rerun 里「手在线程池、姿态在主线程」
的写法不用改。

置信度换算（写清楚，阈值都在下游）：
  • visibility = RTMW 该点的 SimCC 置信度（两轴峰值取小，和 mmpose 一致）乘 GPU_VIS_GAIN
    （默认 1.67，即 RTMW 0.3 ↔ 下游 MIN_VIS 0.5），再截到 [0,1]。
  • presence = 点落在画面内（留 EDGE 余量）为 1，否则 0。RTMW 对画面外的点也会给坐标，
    这一项让下游 MIN_PRESENCE 照样能把飞出画面的点筛掉。
  • 「是不是人」这一关在检测器上卡：YOLOX 人框分数 < GPU_DET_THR（默认 0.5）的框不出人。
    MediaPipe 那边对应 POSE_CONF=0.8；两边分数的含义不同，数值不能直接比，阈值见回执实测。
  • 第二道：框内 17 个身体点平均置信度 < GPU_PERSON_THR（默认 0.3）的人整个丢掉。
  • 对不上 MediaPipe 的下标（嘴角 9/10、眼内外角 1/3/4/6、手上 17~22）visibility/presence 填 0。
  • 手：RTMW 手上 21 点的平均置信度 ≥ GPU_HAND_THR（默认 0.5）才算检出这只手。

环境变量：
  GPU_POSE_MODEL  RTMW onnx 文件名（models/gpu/ 下），默认 rtmw-dw-x-l 256×192
  GPU_DET_MODEL   YOLOX onnx 文件名，默认 yolox_tiny humanart 416×416（有 _nonms 版就用它）
  GPU_DET_EP      检测器的 EP，默认 _nonms 版 trt、原版 cuda
  GPU_DET_EVERY   每几帧跑一次检测器（默认 3）；中间帧用上一帧关键点外扩成框
  GPU_EP          trt | cuda | cpu（默认 trt；失败自动退到下一个）
"""
import os
import threading

import cv2
import numpy as np

from .paths import default_models_dir

HERE = os.path.dirname(os.path.abspath(__file__))
# GPU 模型不随感知包复制；需要时放到 VR_safe/models/gpu/（bootstrap 负责）。
GPU_DIR = os.path.join(default_models_dir(), "gpu")
POSE_ONNX = os.environ.get(
    "GPU_POSE_MODEL", "rtmw-dw-x-l_simcc-cocktail14_270e-256x192_20231122.onnx")
# 检测器优先用切掉 NMS 的版本（models/cut_yolox_nms.py 生成），整个进 TensorRT；
# 没有就用原版（带 NMS）走 CUDA EP，慢一些（11-16 实测纯推理 5 ms vs 17 ms）。
_DET_NONMS = "yolox_tiny_8xb8-300e_humanart-6f3252f9_nonms.onnx"
_DET_ORIG = "yolox_tiny_8xb8-300e_humanart-6f3252f9.onnx"
DET_ONNX = os.environ.get("GPU_DET_MODEL") or (
    _DET_NONMS if os.path.exists(os.path.join(GPU_DIR, _DET_NONMS)) else _DET_ORIG)
DET_THR = float(os.environ.get("GPU_DET_THR", "0.5"))
NMS_IOU = 0.45            # 只对切掉 NMS 的检测模型生效（取 rtmlib 的默认值）
HAND_THR = float(os.environ.get("GPU_HAND_THR", "0.5"))
# 第二道「是不是人」：框里 17 个身体点的平均置信度低于这个，整个人丢掉。
# 11-16 冒烟：真人（含弯腰半身）0.44~0.83，误检框（黑色软管/货架）0.12~0.14。
PERSON_THR = float(os.environ.get("GPU_PERSON_THR", "0.3"))
# RTMW 的置信度普遍比 MediaPipe 的 visibility 低（真人躯干常在 0.3~0.8，MediaPipe 报 ~1.0）。
# 乘 1.67 让 RTMW 0.3（mmpose/rtmlib 默认的关键点可见阈值）对上下游 MIN_VIS=0.5。
# 11-16 在 61 帧真人素材上：系数 1.0 时每人可见点 ≥12 的只有 15/57，1.67 时 49/57；
# 和 MediaPipe 的「可见/不可见」一致率 1.0~2.0 之间都是 81~83%，不敏感。
VIS_GAIN = float(os.environ.get("GPU_VIS_GAIN", "1.67"))
DET_EVERY = max(1, int(os.environ.get("GPU_DET_EVERY", "3")))
EP = os.environ.get("GPU_EP", "trt")
# 检测器单独选 EP：原版 YOLOX 的 onnx 里带 NMS，NMS 之后那段的张量长度随检出框数变化，
# 放进 TensorRT 会因为形状变了反复重编引擎（11-16 实测每次卡 2.9 s），所以原版只能走 CUDA EP；
# 切掉 NMS 的版本是静态形状，走 TensorRT。
DET_EP = os.environ.get("GPU_DET_EP") or ("trt" if DET_ONNX.endswith("_nonms.onnx") else "cuda")
POSE_BGR = os.environ.get("GPU_POSE_BGR", "0") == "1"   # RTMW 输入通道顺序（实测见回执）
EDGE = 0.0           # presence 判定的画面余量（归一化），0 = 只要在画面内
TRACK_KPT_THR = 0.3  # 用上一帧关键点外扩成框时，只用置信度够的点

# COCO-WholeBody 下标 → MediaPipe BlazePose 33 点下标。左右都是被拍的人自己的左右，
# 两边约定一致（COCO 5 = left_shoulder，MediaPipe 11 = left_shoulder）。
WB2MP = {0: 0, 1: 2, 2: 5, 3: 7, 4: 8, 5: 11, 6: 12, 7: 13, 8: 14, 9: 15, 10: 16,
         11: 23, 12: 24, 13: 25, 14: 26, 15: 27, 16: 28,
         17: 31, 20: 32, 19: 29, 22: 30}
LHAND = slice(91, 112)    # WholeBody 左手 21 点，顺序同 MediaPipe（腕、拇指 4、食指 4…）
RHAND = slice(112, 133)

MEAN = np.array([123.675, 116.28, 103.53], np.float32)
STD = np.array([58.395, 57.12, 57.375], np.float32)


class LM:
    """MediaPipe NormalizedLandmark 的替身。"""
    __slots__ = ("x", "y", "z", "visibility", "presence")

    def __init__(self, x=0.0, y=0.0, z=0.0, visibility=0.0, presence=0.0):
        self.x, self.y, self.z = x, y, z
        self.visibility, self.presence = visibility, presence

    def __repr__(self):
        return (f"LM(x={self.x:.3f}, y={self.y:.3f}, vis={self.visibility:.2f}, "
                f"pres={self.presence:.2f})")


class Result:
    def __init__(self, poses, hands, raw=None):
        self.pose_landmarks = poses
        self.pose_world_landmarks = []
        self.hand_landmarks = hands
        self.handedness = []
        self.raw = raw          # (kpts[N,133,2] 像素, scores[N,133], boxes[N,5])，调试用


def _session(path, trt_profile=None, ep=None):
    import onnxruntime as ort
    ort.set_default_logger_severity(3)
    cache = os.path.join(GPU_DIR, "trt_cache")
    os.makedirs(cache, exist_ok=True)
    trt = {"trt_fp16_enable": True, "trt_engine_cache_enable": True,
           "trt_engine_cache_path": cache, "trt_timing_cache_enable": True,
           "trt_timing_cache_path": cache}
    if trt_profile:
        trt.update(trt_profile)
    chain = {"trt": [("TensorrtExecutionProvider", trt), "CUDAExecutionProvider",
                     "CPUExecutionProvider"],
             "cuda": ["CUDAExecutionProvider", "CPUExecutionProvider"],
             "cpu": ["CPUExecutionProvider"]}[ep or EP]
    so = ort.SessionOptions()
    so.log_severity_level = 3
    # 省 CPU：模型几乎全在 GPU 上，CPU 线程只管调度。默认 onnxruntime 会按核数开线程并忙等（spin），
    # 白白吃 CPU；这里限 1 个线程、关掉忙等。GPU_ORT_SPIN=1 恢复默认行为做对照。
    if os.environ.get("GPU_ORT_SPIN", "0") != "1":
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")
        so.add_session_config_entry("session.inter_op.allow_spinning", "0")
    return ort.InferenceSession(path, so, providers=chain)


def _affine(center, box_w, out_w, out_h):
    """以 center 为中心、宽 box_w（高按 out_h/out_w 比例）的框 → out_w×out_h，无旋转。"""
    s = out_w / box_w
    return np.array([[s, 0, out_w * 0.5 - center[0] * s],
                     [0, s, out_h * 0.5 - center[1] * s]], np.float32)


class WholeBody:
    """YOLOX + RTMW，一帧出所有人的 133 点。线程安全，同一 ts 只算一次。"""

    def __init__(self, max_persons=3):
        self.max_persons = max_persons
        # OpenCV 默认按核数开线程，裁图/缩放这种小活全靠它们空转：11-16 实测 GPU 后端每帧
        # CPU 49 ms，其中约 2/3 是 OpenCV 线程池；限成 1 个线程后每帧 17 ms，墙钟不变。
        # 注意这是进程级设置，会影响同进程里其它 cv2 调用；GPU_CV_THREADS=0 表示不动它。
        nt = int(os.environ.get("GPU_CV_THREADS", "1"))
        if nt > 0:
            cv2.setNumThreads(nt)
        self.det = _session(os.path.join(GPU_DIR, DET_ONNX), ep=DET_EP)
        di = self.det.get_inputs()[0]
        self.det_in = di.name
        self.det_hw = (int(di.shape[2]), int(di.shape[3]))
        self.det_nonms = self.det.get_outputs()[0].shape[-1] == 4
        mp_ = max_persons
        self.pose = _session(os.path.join(GPU_DIR, POSE_ONNX), {
            "trt_profile_min_shapes": "input:1x3x256x192",
            "trt_profile_max_shapes": f"input:{mp_}x3x256x192",
            "trt_profile_opt_shapes": "input:1x3x256x192"})
        pi = self.pose.get_inputs()[0]
        self.pose_in = pi.name
        self.pose_hw = (int(pi.shape[2]), int(pi.shape[3]))   # (256, 192)
        self.lock = threading.Lock()
        self.cache_ts = None
        self.cache = None
        self.n = 0
        self.prev_boxes = None
        self.providers = (self.det.get_providers()[0], self.pose.get_providers()[0])

    # ── 检测 ──
    def detect_people(self, bgr):
        h, w = bgr.shape[:2]
        ih, iw = self.det_hw
        r = min(ih / h, iw / w)
        pad = np.full((ih, iw, 3), 114, np.uint8)
        rs = cv2.resize(bgr, (int(w * r), int(h * r)), interpolation=cv2.INTER_LINEAR)
        pad[:rs.shape[0], :rs.shape[1]] = rs
        x = pad.transpose(2, 0, 1)[None].astype(np.float32)
        out = self.det.run(None, {self.det_in: x})
        if self.det_nonms:
            # 切掉 NMS 的模型（models/cut_yolox_nms.py）：boxes[1,N,4]、scores[1,C,N]，NMS 在这里做
            bx, sc = out[0][0], out[1][0].max(0)
            m = sc >= DET_THR
            bx, sc = bx[m], sc[m]
            if len(bx):
                xywh = [[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in bx]
                # NMSBoxes 一个都不留时返回空元组，np.array(()) 是 float，拿去当下标会抛
                # IndexError（11-16 实测把整个检测进程弄崩了），所以强制成整数
                keep = np.asarray(cv2.dnn.NMSBoxes(xywh, sc.tolist(), DET_THR, NMS_IOU),
                                  dtype=np.int64).reshape(-1)
                dets = np.concatenate([bx[keep], sc[keep, None]], 1).astype(np.float32)
            else:
                dets = np.zeros((0, 5), np.float32)
        else:
            dets = out[0][0]
        if len(dets) == 0:
            return np.zeros((0, 5), np.float32)
        dets = dets[dets[:, 4] >= DET_THR].copy()
        dets[:, :4] /= r
        dets = dets[np.argsort(-dets[:, 4])][:self.max_persons]
        return dets

    def boxes_from_kpts(self, kpts, scores, w, h):
        out = []
        for k, s, prev in zip(kpts, scores, self.prev_boxes):
            m = s[:23] > TRACK_KPT_THR          # 身体 + 脚，不看脸手的细点
            if m.sum() < 4:
                return None                      # 跟丢了，下一帧重新检测
            p = k[:23][m]
            x0, y0 = p.min(0)
            x1, y1 = p.max(0)
            cx, cy, bw, bh = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
            bw, bh = bw * 1.2 + 10, bh * 1.2 + 10
            out.append([max(0, cx - bw / 2), max(0, cy - bh / 2),
                        min(w - 1, cx + bw / 2), min(h - 1, cy + bh / 2), prev[4]])
        return np.array(out, np.float32)

    # ── 关键点 ──
    def keypoints(self, rgb, boxes):
        oh, ow = self.pose_hw
        crops, cs = [], []
        for b in boxes:
            x0, y0, x1, y1 = b[:4]
            c = np.array([(x0 + x1) / 2, (y0 + y1) / 2], np.float32)
            bw, bh = (x1 - x0) * 1.25, (y1 - y0) * 1.25
            if bw > bh * ow / oh:
                bh = bw * oh / ow
            else:
                bw = bh * ow / oh
            M = _affine(c, bw, ow, oh)
            crop = cv2.warpAffine(rgb, M, (ow, oh), flags=cv2.INTER_LINEAR)
            if POSE_BGR:
                crop = crop[..., ::-1]
            crops.append(((crop.astype(np.float32) - MEAN) / STD).transpose(2, 0, 1))
            cs.append((c, bw))
        x = np.stack(crops).astype(np.float32)
        sx, sy = self.pose.run(None, {self.pose_in: x})
        ix, iy = sx.argmax(-1), sy.argmax(-1)
        vx, vy = sx.max(-1), sy.max(-1)
        score = np.minimum(vx, vy)
        kx, ky = ix / 2.0, iy / 2.0                    # SimCC split ratio 2
        kpts = np.zeros(score.shape + (2,), np.float32)
        for n, (c, bw) in enumerate(cs):
            s = bw / ow
            kpts[n, :, 0] = c[0] + (kx[n] - ow * 0.5) * s
            kpts[n, :, 1] = c[1] + (ky[n] - oh * 0.5) * s
        return kpts, score

    def infer(self, rgb, ts_ms=None):
        with self.lock:
            if ts_ms is not None and ts_ms == self.cache_ts:
                return self.cache
            h, w = rgb.shape[:2]
            boxes = None
            if (self.prev_boxes is not None and len(self.prev_boxes)
                    and self.n % DET_EVERY != 0 and self.cache is not None):
                kp, sc, _ = self.cache.raw
                boxes = self.boxes_from_kpts(kp, sc, w, h)
            if boxes is None:
                boxes = self.detect_people(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            self.n += 1
            if len(boxes) == 0:
                kpts = np.zeros((0, 133, 2), np.float32)
                score = np.zeros((0, 133), np.float32)
            else:
                kpts, score = self.keypoints(rgb, boxes)
                keep = score[:, :17].mean(1) >= PERSON_THR
                kpts, score, boxes = kpts[keep], score[keep], boxes[keep]
            self.prev_boxes = boxes
            res = self._to_mp(kpts, score, boxes, w, h)
            self.cache_ts, self.cache = ts_ms, res
            return res

    @staticmethod
    def _to_mp(kpts, score, boxes, w, h):
        poses, hands = [], []
        for k, s in zip(kpts, score):
            lms = [LM() for _ in range(33)]
            for wi, mi in WB2MP.items():
                x, y = float(k[wi, 0]) / w, float(k[wi, 1]) / h
                v = min(1.0, max(0.0, float(s[wi]) * VIS_GAIN))
                inside = (EDGE <= x <= 1 - EDGE) and (EDGE <= y <= 1 - EDGE)
                lms[mi] = LM(x, y, 0.0, v, 1.0 if inside else 0.0)
            poses.append(lms)
            for sl in (LHAND, RHAND):
                if float(s[sl].mean()) >= HAND_THR:
                    hands.append([LM(float(p[0]) / w, float(p[1]) / h, 0.0, 1.0, 1.0)
                                  for p in k[sl]])
        return Result(poses, hands, (kpts, score, boxes))


def _rgb(img):
    """接受 mp.Image（SRGB）或 RGB ndarray。"""
    if hasattr(img, "numpy_view"):
        a = img.numpy_view()
        return a[..., :3] if a.ndim == 3 and a.shape[2] == 4 else a
    return img


_SHARED = {}
_SHARED_LOCK = threading.Lock()


def shared_model(max_persons=3):
    with _SHARED_LOCK:
        if max_persons not in _SHARED:
            wb = WholeBody(max_persons)
            print(f"[gpu_pose] 检测 {DET_ONNX} @ {wb.providers[0]}；关键点 {POSE_ONNX} @ "
                  f"{wb.providers[1]}；每 {DET_EVERY} 帧检测一次", flush=True)
            _SHARED[max_persons] = wb
        return _SHARED[max_persons]


class _Pose:
    def __init__(self, wb):
        self.wb = wb

    def detect_for_video(self, img, ts_ms):
        r = self.wb.infer(_rgb(img), ts_ms)
        return Result(r.pose_landmarks, [], r.raw)

    detect = lambda self, img: self.detect_for_video(img, None)   # noqa: E731

    def close(self):
        pass


class _Hands(_Pose):
    def detect_for_video(self, img, ts_ms):
        r = self.wb.infer(_rgb(img), ts_ms)
        return Result([], r.hand_landmarks, r.raw)


def make_gpu_pose(max_persons=3):
    return _Pose(shared_model(max_persons))


def make_gpu_hands(max_persons=3):
    return _Hands(shared_model(max_persons))
