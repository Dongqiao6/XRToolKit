# camera_zone — 工位占用检测节点

人机共存工位的感知节点。基于 RGB-D 相机与人体姿态估计，实时判定指定工位区域
的占用状态，并以 ROS 2 话题发布，供运动监督节点决策。

- **版本** 2026-11-11
- **运行环境** Ubuntu 22.04 / ROS 2 Humble / Python 3.10
- **硬件** Orbbec Gemini 335L 或 Intel RealSense D4xx；纯 CPU 推理，无需 GPU
- **接口** `/workcell/occupancy`（`workcell_interfaces/msg/ZoneState`）

本节点在五节点架构中承担感知职责，默认不直接干预机器人运动。为便于在监督节点
尚未就绪时进行链路验证，提供可选的直接中断模式（§5）。

## 快速开始

```bash
cd <本目录>          # 本机：/home/admin/Documents/dj_main/XRToolKit/camera_zone

./start.sh          # 主演示：相机 + rerun 三维窗口 + 真机关节角
./stop.sh           # 关（相机会正常关流）

./rosrun.sh camera_zone.py --show            # 只发 ROS 话题，同时开预览窗
./rosrun.sh camera_zone.py --fake occupied   # 无相机联调
```

> 目录可整体搬移：模型、配置、URDF 均按脚本自身路径定位。`start.sh` / `stop.sh`
> 的 pid 与日志按目录区分（`/tmp/camera_zone-<目录哈希>.pid|log`），同一台机器上
> 跑多份副本互不干扰。

---

## 目录

1. [接口包编译](#1-接口包编译)
2. [无相机联调](#2-无相机联调)
3. [完整部署](#3-完整部署)
4. [消息规范](#4-消息规范)
5. [直接中断模式（可选）](#5-直接中断模式可选)
6. [待确认事项](#6-待确认事项)
7. [功能范围与已知限制](#7-功能范围与已知限制)
8. [附属工具](#8-附属工具)

---

## 1. 接口包编译

仅依赖 ROS 2 Humble，不需要相机硬件与 Python 推理依赖。

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select workcell_interfaces
source install/setup.bash
```

验证：

```bash
ros2 interface show workcell_interfaces/msg/ZoneState
```

---

## 2. 无相机联调

建议先以模拟数据验证暂停—恢复时序，确认无误后再接入真实相机。
以下模式仅依赖 `rclpy` 与 `workcell_interfaces`，不加载相机驱动与推理模型。

```bash
# 时序脚本：空闲 8 s → 占用 6 s → 空闲 6 s → 不可判定 3 s，循环
./rosrun.sh camera_zone.py --fake script

# 手动控制：c=无人  o=占用  u=不可判定  n=邻近
./rosrun.sh camera_zone.py --fake keys

# 恒定状态
./rosrun.sh camera_zone.py --fake occupied
```

> 必须经 `rosrun.sh`：系统 `python3` 没有 `rclpy`，且其 NumPy 与 MediaPipe 冲突；
> `rosrun.sh` 会先加载 ROS 环境、再发到 domain 42。

订阅端验证（须与发布端同域）：

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 topic echo /workcell/occupancy
```

> **域隔离**：本节点默认 `ROS_DOMAIN_ID=42` 且 `ROS_LOCALHOST_ONLY=1`，
> 与本地局域网 domain 0 上的真机控制话题隔离。联调前需确认双方域号一致。

---

## 3. 完整部署

### 3.1 依赖安装

> 仓库不提交 `.venv`（虚拟环境属于本机环境）。目标机按本节命令重建
> Jetson aarch64 环境（numpy / opencv / mediapipe / pyorbbecsdk2 / rerun /
> onnxruntime），再从 §3.2 开始。

```bash
python3 -m venv .venv
./.venv/bin/pip install numpy opencv-python mediapipe pyorbbecsdk2
```

RealSense 相机将 `pyorbbecsdk2` 替换为 `pyrealsense2`；两家 SDK 的差异由
`camera_backend.py` 封装，上层代码无需修改。

MediaPipe 模型文件 `pose_landmarker_full.task` 与 `hand_landmarker.task`
置于 `models/` 目录，可从 MediaPipe 官方发布页获取，或联系本节点作者提供。

### 3.2 区域标定

```bash
# 步骤一：拟合工位所在平面
./.venv/bin/python zone_tune.py --refit --range 1.5 5 --save

# 步骤二：操作人员站立于工位中心，自动对齐区域位置
./.venv/bin/python zone_tune.py --at-person --save
```

区域参数可独立调整，无需重新拟合平面：

| 参数 | 含义 |
|---|---|
| `--size 长 宽` | 区域尺寸（米） |
| `--offset d1 d2` | 平面内平移（米） |
| `--rotate 角度` | 平面内旋转（度） |
| `--height 高度` | 判定柱体高度（米），默认 2.2 |
| `--near 距离` | 邻近判定阈值（米），默认 0.5 |
| `--densest` | 选取点云密度最高的平面，而非最低平面 |

不带 `--save` 时仅输出预览，不写入配置。

> **平面拟合注意**：相机架设于台面之上时，近处台面点云密度最高，直接拟合
> 会误选台面。`--range 1.5 5` 限定参与拟合的深度区间，可排除近场干扰；
> 程序默认选取**最低**的水平面作为地面。若工位区域位于台面而非地面，使用
> `--densest`。

### 3.3 运行

```bash
./rosrun.sh camera_zone.py --show
```

`rosrun.sh` 用于解决 ROS 2 与虚拟环境的共存问题：先加载 ROS 环境变量
（`AMENT_PREFIX_PATH` / `LD_LIBRARY_PATH` / `PYTHONPATH`），再以虚拟环境的
解释器执行。单独激活虚拟环境会导致 `rclpy` 的 C 扩展无法加载；
单独加载 ROS 环境则会使用系统 Python，其 NumPy 版本不满足 MediaPipe 要求。

主演示走 `start.sh`，它等价于：

```bash
./.venv/bin/python -u zone_rerun.py --robot --joints-live --zone-mode reach
```

`--joints-live` 默认只读 `192.168.71.37` / `192.168.71.36` 的 10000 端口
（不向控制器发任何字节）。机器人不在线时用 `./.venv/bin/python zone_rerun.py
--robot --animate` 或去掉 `--robot`。

---

## 4. 消息规范

**话题** `/workcell/occupancy`　**类型** `workcell_interfaces/msg/ZoneState`

| 字段 | 类型 | 说明 |
|---|---|---|
| `state` | `uint8` | `CLEAR=0` 无人 / `OCCUPIED=1` 占用 / `UNKNOWN=2` 不可判定 |
| `zone_id` | `string` | 工位标识，单工位固定为 `"default"` |
| `near` | `bool` | 人员位于区域外但接近边界，供降速决策使用 |
| `edge_m` | `float32` | 至区域边界距离（米），区内为正，区外为负 |
| `evidence` | `uint8` | 判定依据：躯干 / 四肢 / 头部 / 目标丢失 |
| `reason` | `string` | 人类可读的判定说明，供界面与日志使用 |
| `detail_json` | `string` | 全部中间量，JSON 格式 |

**QoS** `RELIABLE` + `TRANSIENT_LOCAL`，深度 1。订阅端延迟启动亦可立即获取
当前状态，无需等待下一次状态变化。

**退出行为** 节点退出前主动发布一条 `UNKNOWN`，避免 `TRANSIENT_LOCAL` 留存
的 `CLEAR` 使订阅端误认为监测仍在进行。

### 4.1 内部判定档位与三值映射

本节点内部维护五档判定，映射至接口三值如下：

| 内部档位 | 映射 | 依据 |
|---|---|---|
| 空闲 | `CLEAR` | — |
| 邻近 | `CLEAR` + `near=true` | 人员位于区域外，未占用工位；降速策略由监督节点决定 |
| 不可判定 | `UNKNOWN` | 遮挡、超量程或深度数据缺失，按最不利情况处理 |
| 肢体侵入 | `OCCUPIED` | — |
| 占用 | `OCCUPIED` | — |

状态升档即时生效，降档需连续 12 帧满足条件且每次仅降一级，以抑制边界抖动。

---

## 5. 直接中断模式（可选）

撤销运动许可属于 `workcell_supervisor` 的职责。在监督节点就绪前，可启用
`--direct-permit` 由本节点直接发布中断指令，以完成端到端链路验证：

```bash
./rosrun.sh camera_zone.py --direct-permit --show
```

启用后额外发布：

| 话题 / 服务 | 类型 | 触发条件 |
|---|---|---|
| `/workcell/permit` | `MotionPermit` | 占用 / 肢体侵入 / 不可判定 → `allow=false` |
| `/workcell/speed_scale` | `std_msgs/Float32` | 邻近 → `0.3`；正常 → `1.0`；许可撤销 → `0.0` |
| `/workcell/confirm_resume` | `ConfirmResume`（服务） | 人工确认后恢复许可 |

### 5.1 安全约束

1. **撤销即时，恢复需人工确认。** 人员离开区域不触发自动恢复。区域清空仅为
   接受确认请求的前提条件，不构成恢复动作本身。
2. **不可判定按最不利情况处理。** 遮挡或深度缺失导致无法判定时，同样撤销许可。
3. **区域非空时拒绝恢复请求。** 服务返回 `accepted=false`，`message` 说明原因。

### 5.2 人工确认

```bash
ros2 service call /workcell/confirm_resume \
  workcell_interfaces/srv/ConfirmResume "{confirmed_by: 操作员姓名, note: ''}"
```

降速比例由环境变量 `SLOW_SCALE` 配置，默认 `0.3`。
模拟数据模式同样支持 `--direct-permit`，可在无相机条件下验证完整链路。

### 5.3 验证记录（2026-11-11）

| 步骤 | 输出 | 结果 |
|---|---|---|
| 人员进入区域 | `allow=false`、`reason="区域被占"` | 许可撤销 |
| 人员离开区域 | `allow=false` 保持 | 未自动恢复 |
| 人工确认 | `accepted=true` → `allow=true`、`speed_scale=1.0` | 许可恢复 |
| 人员位于邻近区 | `speed_scale=0.3`、`allow=true` | 降速而不停机 |

> 监督节点上线后关闭 `--direct-permit` 即可，本节点恢复为仅发布状态。
> 降速采用独立话题而非扩展 `MotionPermit` 字段，以 `workcell_supervisor`
> 一侧的消息定义为准。

---

## 6. 待确认事项

1. **消息字段。** 三值枚举依照贵方定义；`near` / `edge_m` / `evidence` 为本节点
   补充字段——三值无法表达「人员在区域外但接近边界」这一中间状态，监督节点
   缺此信息则无法实施降速。`MotionPermit`、`ArmExecutionState`、`ConfirmResume`
   在 `ros_ws/src/workcell_interfaces/` 下已有初版定义，**以贵方定义为准**，
   如有冲突请以贵方为基准修改本侧。

2. **ROS 域与话题命名。** 本侧默认 domain 42，需确认双方一致。

3. **发布频率。** 当前跟随相机帧率，21~29 FPS（启用多人跟踪与机械臂动画时
   取下限）。如需限频，使用 `--rate 10`。

---

## 7. 功能范围与已知限制

### 7.1 已实现

- **三维体积占用判定。** 判定区域为「底面多边形 × 高度 2.2 m」的柱体，人体
  任一可见关键点落入即判定为占用。不依赖足部可见性——躯干在绝大多数机位下
  均可稳定检出。
- **不可判定状态上报。** 遮挡、超量程、深度数据缺失时上报 `UNKNOWN`，
  按最不利情况处理。
- **静态物体误检抑制。** 基于运动门控，空场景实测误报率 0%（对照：仅提高
  姿态检测阈值至 0.8 时为 1%，默认阈值 0.5 时为 82%）。
- **多人跟踪。** 逐人独立判定，取风险最高者作为输出结论。
- **骨架约束反推。** 下肢部分遮挡时，依据父关节位置、个体骨长先验与图像方向
  反算关节三维位置。遮挡场景下膝关节可用帧数由 2/534 提升至 85/534。
  可通过 `PRIOR=0` 关闭。
- **CPU 推理。** 无需独立显卡。

### 7.2 已知限制

- **安全响应时间未测定。** 指标要求小于 200 ms。本节点单帧处理耗时
  34~48 ms，端到端时延需在与监督节点、臂驱动联调后测量。
- **相机至机器人基座外参未实测标定。** 当前采用 `sensors.json` 中的设计值，
  该文件自身标注 `extrinsics_calibrated: false`。`robot_frames.align_to_floor()`
  可依据实测地面校正高度与俯仰（实测校正量 0.13 m / 8.8°），但**无法校正
  水平朝向**——地面具有旋转对称性，不含朝向信息。涉及「人员至机械臂距离」
  的计算须先完成实地标定。
- **相机位姿变更后需重新标定区域。** 相机移动后地面平面与机器人位姿均失效，
  须重新执行 §3.2。

---

## 8. 附属工具

以下文件不参与主链路，供调试与可视化使用。

| 文件 | 用途 |
|---|---|
| `zone_rerun.py` | 三维可视化（需 `pip install rerun-sdk`）。`--zone-mode reach` 将保护范围设为机械臂可达空间 |
| `jaka_model.py` | JAKA S5 正运动学、连杆包络、可达空间计算。可达半径解析上界 1.047 m，随机轨迹采样最远 0.966 m |
| `robot_frames.py` | 相机与机器人基座间坐标变换，含基于实测地面的位姿校正 |
| `clip_io.py` `rec_clip.py` `replay_ab.py` | 原始帧录制与离线回放。用于算法改动前后的对照评估 |

> **对照评估须使用同一段录制数据。** 分两次录制进行对照不成立——操作人员的
> 站位与动作无法复现，差异来源不可归因。录制格式为彩色 JPEG + 深度 16 位 PNG
> （毫米单位），逐帧落盘，内存占用与时长无关。
>
> JAKA S5 的 URDF、37 个网格与 `sensors.json` 已随包附带（`jaka_s5_sim/`，
> 约 5 MB，取自 `jaka_s5_mujoco` 0.3.0 仿真包），`zone_rerun.py` 开箱即可渲染
> 机器人模型。运行示例：
>
> ```bash
> pip install rerun-sdk
> ./.venv/bin/python zone_rerun.py --robot --animate --zone-mode reach
> ```

## 锁频（11-16 加）

`start.sh` 每次启动会检查 CPU/GPU 是否锁在满频，没锁就尝试 `sudo -n /usr/bin/jetson_clocks`。
重启后锁频会失效。要让 start.sh 能自动锁，需在 JAKA 本机终端装一次免密规则（只放行 jetson_clocks 这一条命令）：

    echo 'admin ALL=(root) NOPASSWD: /usr/bin/jetson_clocks' | sudo tee /etc/sudoers.d/jetson_clocks >/dev/null
    sudo chmod 440 /etc/sudoers.d/jetson_clocks
    sudo visudo -cf /etc/sudoers.d/jetson_clocks      # 应输出 parsed OK

不想锁：`NO_CLOCKS=1 ./start.sh`。恢复锁频前的设置：`sudo jetson_clocks --restore /home/admin/jetson_clocks_orig.conf`。
