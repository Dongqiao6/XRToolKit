# Vision_Teleop — D435 视觉遥操作 JAKA S5 双臂

用一台 RealSense D435 看操作者的上半身，把腕部动作映射成 JAKA S5 双臂的
TCP 目标位姿。运动层、限幅、离合、急停语义沿用 Quest 3 demo 里已经验证过的
`rc_ctrl_node`，本工程只替换"输入源"。

**本工程与 `demo/` 完全独立**：不 import demo 的任何 Python 模块，不共用
`.venv`，不共用 ROS 包。只通过 ROS 2 话题对接，话题统一加 `/vision` 前缀，
两个工程同时运行时不会串台。

```
D435 ──► PoseLandmarker(33 点) ──► 腕点 + 前臂方向×8cm ──► 工具点
   │                                                     │
   └──► 腕点裁剪 256×256 ──► HandLandmarker ──► 捏合/握拳 │
                                                         ▼
                     相机系 ──(标定 R)×scale──► 机器人 base 系
                                                         │
                              30 fps 目标 ──► 125 Hz 插值 ▼
                    /vision/rc_ctrl/{left,right}_target + /button
                                                         │
                                          vision_rc/rc_ctrl_node ×2
                                                         │
                                            servo_p(INCR) ──► JAKA S5
```

## 目录

```
Vision_Teleop/
├── teaching/            参考代码（不参与运行，不 import）
├── models/              pose_landmarker_full.task / hand_landmarker.task
├── config/
│   ├── vision.yaml      相机、模型、频率、窗口
│   ├── mapping.yaml     相机→base 旋转、尺度、工具点偏移（标定写这里）
│   └── safety.yaml      手势阈值、限速、工作区
├── scripts/
│   ├── download_models.sh
│   ├── calibrate.py     三步交互标定
│   └── selftest.py      离线自检（不需要相机/ROS）
├── src/vision_bridge/   Python：感知 + 映射 + 插值 + 界面
├── src/vision_rc/       C++：复制自 demo 的 rc_ctrl_node + PGI-140 + JAKA SDK
└── env.sh
```

## 安装

```bash
cd Vision_Teleop
/usr/bin/python3.10 -m venv .venv                      # 已创建
.venv/bin/pip install -r requirements.txt
./scripts/download_models.sh
```

`env.sh` 会把 ROS 2 Humble、`src/vision_bridge` 和 venv 一起加载：

```bash
source env.sh
```

venv 刻意不使用 `--system-site-packages`，`rclpy` 通过 `PYTHONPATH` 从
`/opt/ros/humble` 注入，这样 venv 里的 numpy 不会被系统版本覆盖。

## 编译

```bash
source env.sh
colcon build --symlink-install --packages-select vision_rc vision_bridge
source install/setup.bash
```

## 手势阈值调参

不同人的手型、站姿会让捏合距离不一样，**第一次用先跑一次调参**：

```bash
source env.sh
python scripts/tune_gesture.py --write
```

按屏幕提示：捏合按 `1`、张开按 `2`、握拳按 `3`，每种按 3–5 次，`q` 结束。
脚本会统计每类的分位数，算出 `pinch_close` / `pinch_open` / 握拳阈值并写回
`config/safety.yaml`。不加 `--write` 只打印建议值。

手部裁剪是**自适应**的：以掌心（腕点与食指点的中点）为中心，边长取手部尺寸的
3.5 倍（96–320 px），所以人离相机远近都能保持手在模型输入里占足够像素。

## 坐标系：自动跟随人体，不需要标定

默认 `use_body_frame: true`：每一帧用骨架估计相机→机器人 base 的旋转
（肩线定左右、肩→髋定上下，髋部看不到时退化为相机竖直向上），再做 EMA 平滑。
**相机挪位置不用重新标定**，只要操作者面向相机即可。

想关掉这个行为、改用固定的静态旋转（`config/mapping.yaml` 的 `rotation`），
把 `use_body_frame` 设为 `false`。

如果操作者经常侧对相机、或者你希望坐标系完全固定，才需要下面的标定。

## 标定（可选，仅在关闭人体坐标系时需要）

```bash
source env.sh
python scripts/calibrate.py            # 默认用右手
```

按提示把手依次朝相机、朝自己右侧、朝上各移动约 20 cm，每次移动前后按空格采点。
脚本会拟合出相机到机器人 base 系的旋转，写入 `config/mapping.yaml`，并把三个
方向的长度、夹角和行列式打印出来便于判断这次标定是否可信。

相机正对操作者、水平放置时，`mapping.yaml` 里的默认旋转已经可用，标定的收益
主要来自相机俯仰和站位偏移。

## 运行

```bash
source env.sh
ros2 launch vision_rc vision_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36
```

窗口开关在 `config/vision.yaml` 的 `window.enabled`，`start_bridge:=false` 可以只启动伺服层。

只跑 C++ 伺服层（例如先用手工 `ros2 topic pub` 验证机械臂）：

```bash
ros2 launch vision_rc rc_ctrl_dual.launch.py
```

## 操作方式

| 动作 | 效果 |
| --- | --- |
| 双手握拳保持 1 秒 | 进入可遥操作状态（状态条变绿） |
| 左手捏合（拇指+食指） | 左臂跟随左手，松开即冻结 |
| 右手捏合 | 右臂跟随右手，松开即冻结 |
| 双手同时捏合 | 双臂同时跟随，互不影响 |
| ESC | 冻结双臂并退回待命，必须重新握拳进入 |
| q | 退出界面 |

状态条颜色：灰 = 待命（等握拳进入）；橙 = READY（已进入，等捏合）；绿 = 正在跟随；
红 = ESC 冻结，需重新握拳进入。

窗口里蓝色大圆是左手腕、橙色是右手腕，绿色十字是实际映射到 TCP 的"工具点"，
两者差一个 `tool_offset_m`（默认 8 cm，沿前臂方向）。手部方框是喂给
`HandLandmarker` 的裁剪区域。

## 参数

`config/vision.yaml`

| 键 | 说明 |
| --- | --- |
| `camera.serial` | 留空取第一台；插着多台相机时填 D435 序列号 |
| `camera.color/depth` | 分辨率与帧率，彩色默认 1280×720@30 |
| `pose.min_visibility` | 低于该可见度的腕点/肘点直接丢弃 |
| `hand.crop_size` | 手腕裁剪边长，越大手部模型越准、越慢 |
| `publish.rate` | 插值发布频率，默认 125 Hz |
| `camera.serial` | 绑定相机序列号；多台相机时必须填 |
| `camera.log_level` | librealsense 控制台级别，默认 `none`（关掉无害刷屏） |
| `camera.depth_window` | 深度取中值的窗口半径，3 表示 7×7 |
| `smoothing.*` | One Euro 滤波；调大 `min_cutoff` 更跟手但更抖 |
| `quiet_native` | 加载模型时吞掉 LiteRT 的原生横幅日志 |

`config/safety.yaml`

| 键 | 说明 |
| --- | --- |
| `pinch_close` / `pinch_open` | 捏合判定的滞回阈值（按手掌尺寸归一化） |
| `pinch_min_curl` | 手指卷曲到该值以下一律按握拳处理，避免拳头被当成捏合 |
| `gesture_debounce_frames` | 新手势需连续保持几帧才生效，默认 2（约 160 ms） |
| `fist_hold_s` | 握拳进入所需持续时间 |
| `max_step_m` | 每个发布周期的最大位移，默认 4 mm（低于下游 5 mm/cycle 限幅） |
| `workspace_min/max` | base 系工作区，超出即裁剪 |

## 性能（本机 aarch64 实测）

| 环节 | 耗时 |
| --- | --- |
| PoseLandmarker full @1280×720 | 54 ms |
| PoseLandmarker lite @1280×720 | 41 ms |
| HandLandmarker @256 裁剪 | 52 ms |
| 两只手并行 | 35 ms |
| **整帧（pose + 双手并行）** | **约 80 ms，约 12 fps** |

MediaPipe 的 GPU delegate 在官方 aarch64 wheel 里不可用（缺 `kGpuService`），
所以只能跑 CPU。位置流按 12 fps 更新，发布侧按 125 Hz 插值，机械臂运动仍然是
连续的；真正受限的是"手势离合"的响应，约 100 ms。

追求更高帧率可以：把 `pose.model` 换成 `pose_landmarker_lite.task`（快 25%），
或把 `hand.crop_size` 降到 192，或以后把两只手的推理改成独立线程异步跑。

## 与 demo 的边界

* **复制**：`src/vision_rc/` 来自 `demo/src/jaka_driver` 的 `rc_ctrl_node.cpp`、
  `pgi140_*.cpp`、JAKA 头文件与 `libjakaAPI.so`。
* **不复制**：`jaka_driver.cpp`（ROS service 驱动）、`jaka_msgs`、AMR 两个包、
  `xrt_teleop`。
* **避免同时运行**：两个工程都会 `login_in` 同一台控制器并下发 `servo_p`。
  同一时间只允许一个遥操作工程连接机械臂；`/vision` 前缀只保证话题不串，
  不保证控制器安全。

## 故障排查

| 现象 | 处理 |
| --- | --- |
| `No device detected` | 相机被占用；`rs-enumerate-devices` 确认，或填 `camera.serial` |
| 窗口里没有骨架 | 人不在画面里、光线太暗，或 `pose.min_visibility` 过高 |
| 捏合状态来回跳 | 调 `pinch_close` / `pinch_open` 的间距，或增大 `hand.crop_size` |
| 机械臂不动但界面显示跟随 | 检查 grip 轴：左 0 右 1，`grip_press_threshold` 默认 0.6 |
| 动作方向不对 | 重跑 `scripts/calibrate.py`，或临时改 `mapping.yaml` 的 rotation |

## 已知限制

* 位置遥操作（3-DoF），姿态保持当前 TCP；`use_rpy_ctrl:=true` 会启用姿态通道但
  相机方案没有可靠的手掌朝向。
* 单台 D435，人站得离相机太远时手部模型会丢；建议 1.5–2.5 m。
* 骨架和腕点叠加画的是**原始检测结果**，天生会抖；真正发给机械臂的目标经过
  One Euro 滤波（`config/vision.yaml` 的 `smoothing`）。
* 机械臂目标位置约 12 fps 更新、125 Hz 插值发布；捏合响应约 100 ms。
* `teaching/` 只作算法参考（Windows/MuJoCo/Franka 研究代码，85% 模块在本工程用不到）。
