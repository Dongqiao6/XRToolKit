# VR_safe 安全遥操作 — Spec

状态：`ready-for-agent`（设计已对齐，未实现；决策见 `GLOSSARY.md`、`docs/adr/`、`docs/design.md`）

## Problem Statement

操作员用 Quest 3 同时遥操作 JAKA 双臂、PGI 夹爪和 AMR 底盘。现在 `demo` 的遥操作链路和 `camera_zone` 的安全判定是两套互不相识的工程：相机能判断“人进了工位”，但不会让 `demo` 的臂或底盘停；`demo` 也没有统一的许可、锁存、人工复位和审计。操作员无法在一个系统里既让底盘和双臂同时动，又在人进入保护区时让它们一起停。

另外，现有 `camera_zone` 的 ROS 节点把“观察”和“许可”混在同一个进程里，且当前外参标定是 `ok:false`；如果直接复用，后面接第二个传感器、第二台机器人时会被迫改协议。

## Solution

建立一个完全独立的 `VR_safe` 工程，拥有自己的 `.venv`、ROS 2 工作空间、配置、模型和启动脚本，运行期不依赖 `demo/`、`camera_zone/`、`Vision_Teleop/` 的任何代码或产物。

系统分成三层：**观察（Observation）→ 许可（Permit）→ 执行（StopLevel）**。`vr_safe_perception` 和 `vr_safe_teleop` 只产生观察/目标/心跳；`vr_safe_supervisor` 是唯一的许可裁决者，负责锁存、心跳看门狗和人工复位；两个 `vr_safe_arm`（C++，各连一臂）和一个 `vr_safe_chassis` 订阅许可并执行 RUN/SLOW/HOLD。相机判定占用、未知或观察超时，底盘和双臂一起停；恢复必须人工确认，并重新建立跟随。

## User Stories

1. 作为操作员，我希望在一个 launch 里启动双臂、夹爪和底盘，以便不用分别记三套启动命令。
2. 作为操作员，我希望用左摇杆驱动底盘、用 Grip 驱动双臂，以便底盘和机械臂可以同时动。
3. 作为操作员，我希望在同一个界面上看到当前许可、停止原因和是哪个观察源触发的，以便知道为什么机器人不动。
4. 作为操作员，我希望人进入工位时双臂和底盘一起停，以便不会只停一半。
5. 作为操作员，我希望人在区域边缘时系统降速而不是直接停，以便作业不中断。
6. 作为操作员，我希望普通“看不清”不要频繁停机；但人进过区域后跟丢、观察超时或感知源消失时仍然 HOLD，以便兼顾可操作性和失效方向。
7. 作为操作员，我希望相机掉线/深度失效时系统停，以便感知失效不会静默放行。
8. 作为操作员，我希望 Quest/PC-service/Wi-Fi 断开时系统停，以便不会保留最后一条速度。
9. 作为操作员，我希望按左摇杆（`LJ`）触发系统软件急停，以便一键停掉所有运动。
10. 作为操作员，我希望 `Y` 只解除急停按钮的锁存、不直接恢复运动，以便误按后不会立刻动。
11. 作为操作员，我希望区域里还有人时人工确认被拒绝，以便不能绕过安全判断。
12. 作为操作员，我希望人还在旁边时确认后只恢复到慢速，以便不会立刻全速。
13. 作为操作员，我希望恢复后双臂必须重新按 Grip、重新锚定，以便停止期间的手部位移不会被一次性补上。
14. 作为操作员，我希望恢复后底盘必须摇杆回中并重新使能遥控，以便不会因为摇杆还偏着就窜出去。
15. 作为操作员，我希望 HOLD 时夹爪保持最后位置和力、不自动张开，以便不会因为停机器人而掉件。
16. 作为操作员，我希望左/右夹爪锁定仍然在 `X`/`A`，以便不用改肌肉记忆。
17. 作为操作员，我希望右 `B` 的双臂协同模式保留，以便同时移动两臂。
18. 作为操作员，我希望知道当前是 `polygon` 还是 `reach` 保护区，以便理解“为什么这里算危险”。
19. 作为安全工程师，我希望现场标定工位多边形和相机↔基座外参，以便保护区与实际地面一致。
20. 作为安全工程师，我希望外参缺失、`ok:false` 或地面不匹配时生产拒绝启动，以便不会拿一次失败的标定当成功。
21. 作为安全工程师，我希望选择 Orbbec 335L 或 RealSense 作为相机后端，以便换相机不改判定逻辑。
22. 作为安全工程师，我希望知道相机 FOV 的盲区，并且 AMR 自带 lidar/bumper/急停仍是底盘碰撞的主责，以便不夸大成 360° 安全。
23. 作为开发者，我希望新增一个观察源（例如力传感器、门禁、第二个相机）时只实现观察接口，以便不改控制节点。
24. 作为开发者，我希望新增一个执行端（例如 AMR 底盘、第二台机器人）时只订阅许可，以便不改 supervisor。
25. 作为开发者，我希望不接真相机/真机械臂也能跑完整安全闭环，以便在会议室和 CI 里验证。
26. 作为开发者，我希望用同一段录像回放做感知改动前后的对照，以便差异能归因到算法而不是站位。
27. 作为开发者，我希望测试打在许可边界上，而不是 supervisor 的内部函数上，以便重构不碎测试。
28. 作为维护者，我希望审计日志记录许可变化、观察切换、确认、心跳丢失和配置哈希，以便事故后复盘。
29. 作为维护者，我希望用 start/stop 脚本手动启停、默认不开机自启，以便先验证启动顺序和失败路径。
30. 作为维护者，我希望离线 bootstrap 建 venv、装轮子、复制模型、编译工作空间，以便现场没有外网也能装。
31. 作为集成者，我希望 HIL 验收测“观察时间戳→臂/底盘零速”的 p95 ≤ 200 ms，以便不是只看单帧推理时间。
32. 作为集成者，我希望 `start_amr:=false` 时只跑双臂，以便没有 AMR 时也能单独验证。
33. 作为集成者，我希望第一版不包含 AMR 的到站/路径/充电/定位，以便先证明“同时遥操作 + 一起停”。
34. 作为新加入的工程师，我希望有术语表、ADR 和设计文档，以便不用读完全部历史才能改代码。

## Implementation Decisions

### 架构与职责

- 三层契约：`Observation`（感知判断，不含命令）、`Permit`（唯一裁决，`GRANTED/SLOW/REVOKED`）、`StopLevel`（执行层 `RUN/SLOW/HOLD`）。
- `vr_safe_supervisor` 是唯一许可持有者：聚合观察、监控观察/遥操作心跳、锁存、提供人工复位、写审计事件。
- 两个 `vr_safe_arm` 进程，因为 JAKA SDK 每进程一条连接；每个进程只连一臂。
- `vr_safe_chassis` 只做 AMR 遥控、暂停/零速、状态/里程计；不带到站/路径/充电/定位。
- 不建 `workcell_supervisor` 兼容层；旧工程的 `/workcell/*` 命名不进入新系统。

### 接口契约

- 安全消息包 `vr_safe_interfaces`：`SafetyObservation`、`MotionPermit`、`TeleopHeartbeat`、`ArmState`、`ConfirmResume`。
- `SafetyObservation`：来源、区域、四值状态（`CLEAR/NEAR/OCCUPIED/UNKNOWN`）、置信度、证据、明细。
- `MotionPermit`：三值决定、原因、来源、是否锁存、序号。
- `TeleopHeartbeat`：来源、序号。
- 话题统一 `/vr_safe/...`；`permit` 用 `RELIABLE + TRANSIENT_LOCAL`、深度 1；观察用 `RELIABLE`；心跳用 `RELIABLE`（兼容 RELIABLE/BEST_EFFORT 订阅端；实现见 teleop.yaml 的 heartbeat_qos_reliability）。
- AMR 状态消息从 `sr_amr_interfaces` 复制 `SystemState`、`BatteryState` 到独立的底盘接口包；不复制 action。

### 安全策略

- 启动后许可先 `REVOKED`；第一条有效观察为 `CLEAR` 才 `GRANTED`。
- `NEAR → SLOW`（臂和底盘 0.3×）；`OCCUPIED` 或“人进过区后跟丢”→`REVOKED`；普通 `UNKNOWN` 不停止；观察超时/感知源消失/心跳丢失仍 `REVOKED`。
- `REVOKED` 锁存；只能 `confirm_resume` 解除；确认要求当前观察为 `CLEAR` 或 `NEAR`；`NEAR` 只恢复到 `SLOW`。
- `LJ` 是系统软件急停：发 `ESTOP` 观察，supervisor 锁存 `REVOKED`，双臂 HOLD、底盘零速；`Y` 只解除按钮锁存，许可恢复仍要人工确认。
- 人工确认后：双臂必须重新按 Grip 并重新锚定；底盘必须摇杆回中并重新使能遥控。
- HOLD/ SLOW 不改变夹爪指令：保持最后位置/力，不自动张开，也不缩放夹爪速度。

### 控制层

- 双臂：复用 `Vision_Teleop` 的 `vision_rc` 作为控制基线；加许可门、本地 target 看门狗、重离合；保留 Grip 迟滞、5 mm/周期限速、y 向 ±110 mm 限位、碰撞检测、`servo_hold`。
- 底盘：复用 `demo` 的 AMR 遥控子集；`REVOKED` 时抑制操作员速度指令，立即并每 100 ms 发零速；连续 5 次零速失败升级 AMR `emergency_stop`；本地 200 ms 无新 Twist 自动零速。
- `move_pause` 只对 action 有效，第一版遥控不依赖它。

### 感知

- 复用 `camera_zone` 的 `zone_monitor` 判定核心：三维柱体、`BODY_PARTS=upper`、飞点治理、骨架反推、运动门、质量门、`Grader`。
- 五档映射四值：空闲→`CLEAR`、旁边有人→`NEAR`、看不清→`UNKNOWN`、手伸入/占用→`OCCUPIED`。
- 区域支持 `polygon` 和 `reach`，默认 `polygon`。
- 相机默认 Orbbec 335L；RealSense 可选。
- 姿态后端默认 MediaPipe CPU；GPU 可选。
- 标定工具：区域标定、外参标定、2D 预览、录像/回放；生产不带 rerun/table_zones。

### 配置与失败策略

- 配置分成安全策略、遥操作、底盘、感知四份；标定产物保持 `camera_zone` 的原格式。
- 关键默认值：观察超时 300 ms、心跳超时 300 ms、SLOW 0.3×、零速 keepalive 100 ms、本地看门狗 200 ms。
- 生产拒绝无效外参；开发用 `--allow-uncalibrated` 显式降级。
- JAKA 任一臂连接失败中止启动；AMR 连接失败中止启动（`start_amr:=false` 可只跑臂）；相机失败允许启动但许可始终 `REVOKED`。

### 约束

- 保护区在 JAKA base 系，随底盘移动；不做世界系补偿。
- 不宣称相机 360° 覆盖；AMR 控制器自带安全仍是底盘碰撞主责。
- 所有实现只写 `VR_safe/`；`demo/`、`camera_zone/`、`Vision_Teleop/` 只读。

### 相关决策

- `0001` 源码复制换独立
- `0002` 观察/许可/停止三层
- `0003` 相机安全只做到 HOLD
- `0004` 独立 supervisor 持有唯一许可
- `0005` 底盘与 base 系保护区
- `0006` 遥操作心跳
- `0007` 系统软件急停走 supervisor L2
- `0008` 生产拒绝无效外参
- `0009` 独立安全契约包

## Testing Decisions

**什么算好测试**：只测外部行为——给定观察/心跳/按键，许可和执行端命令应该是什么；不测 supervisor 内部状态机字段，不测 MediaPipe 内部函数。时间用可注入的假时钟，录像回放要确定。

**主 seam**：安全契约边界，即“观察/心跳输入 → 许可输出”加“许可 → 执行端命令”。用 fake `SafetySource`、fake `SafetySink` 在同一个 launch 里跑端到端；这是最高的 seam，不穿透 ROS。

**次 seam**：感知的观察输出。用同一段录像回放，比较五档/四值、边距、误报；沿用 `camera_zone` 的 `replay_upper`/`replay_ab` 思路。

**测试模块**：
- supervisor：状态机、锁存、确认拒绝/接受、观察超时、心跳超时、ESTOP、审计。
- perception：录像回放 → 观察；相机失败 → `UNKNOWN`；区域标定。
- teleop：合成输入 → 目标/按键/心跳；`LJ`/`Y` 语义。
- arm sink：fake SDK 下许可门、重离合、target 看门狗。
- chassis sink：fake SRP 下零速 keepalive、Twist 看门狗、5 次失败升级。
- 端到端 dry-run：fake 源/汇 + 真 launch。
- HIL：真机低速、物理急停在旁，测 p95 ≤ 200 ms 和恢复不跳变。

**先例**：`camera_zone` 的录像回放对照、`demo` 的伺服黑匣子；目前没有 supervisor 测试先例，需要新建。

## Out of Scope

- 世界系协调/底盘运动补偿；
- AMR 到站、跟随路径、充电、定位；
- 360° 底盘感知、替代 AMR 自带安全；
- 视觉遥操作（`Vision_Teleop` 保持独立）；
- 远程/云端遥操作、把 DDS 开到局域网；
- GPU 姿态默认、RealSense 默认；
- 开机自启；
- 现场外参重标（属于部署前置工作）。

## Further Notes

- 没有配置 issue tracker；按用户要求，spec 和后续 tickets 以本地 Markdown 保存在 `VR_safe` 下。
- 当前 `extrinsic_calib.json` 是 `ok:false`；生产会拒绝启动，必须先重标或开发用 `--allow-uncalibrated`。
- `xrobotoolkit_sdk 1.1.0` 目前只在 `demo/.venv`，没有现成 wheel；独立安装前需要从 PyPI 下载或从上游打包。
- `sros_sdk_py 1.4.1` 要求 `protobuf>=6.33.2`，与 `camera_zone` 的 `protobuf 5.29.6` 可能冲突；VR_safe 的 venv 必须单独解依赖。
- 200 ms p95 尚未实测；MediaPipe CPU 单帧 34~48 ms，需上机量。
- 模型/轮子约 1 GB，磁盘约 40 GB 可用；`trt_cache` 不复制。
