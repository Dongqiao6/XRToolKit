# XRToolKit

Quest 3 / XRoboToolkit 遥操作与安全控制工作区。包含原始遥操作工程、独立安全层、工位感知、视觉遥操作和官方示例。

## 目录

| 目录 | 实现的功能 | 是否独立 | README |
| --- | --- | --- | --- |
| [`VR_safe/`](VR_safe/) | 独立的人机安全层：Supervisor、双臂/底盘许可门、Quest 遥操作、相机窗口、保护区判定、人工确认与审计。 | **独立**：自己的 `.venv`、ROS 2 工作空间、配置、模型和启动脚本；运行时不 import 兄弟目录，只共享机器人/Quest/相机等硬件。 | [`README.md`](VR_safe/README.md) |
| [`demo/`](demo/) | 原始 Quest 3 → JAKA 双臂、PGI 夹爪和 AMR 遥操作 demo，含伺服黑匣子和诊断工具。 | **独立**：自己的 ROS 2 工作空间、`.venv`、构建/启动脚本；不依赖 `VR_safe` 或 `Vision_Teleop` 的代码。 | [`README.md`](demo/README.md) |
| [`camera_zone/`](camera_zone/) | 工位占用检测与区域标定：RGB-D 相机、人体姿态、区域状态发布、标定/回放工具。 | **独立**：自己的源码、配置、模型和 ROS 接口包；`.venv` 不入库，目标机需重建。`VR_safe` 复制了它的部分感知代码，但运行时不互相 import。 | [`README.md`](camera_zone/README.md) |
| [`Vision_Teleop/`](Vision_Teleop/) | 用 RealSense D435 看操作者上半身，把腕部动作映射成 JAKA S5 双臂 TCP 目标；只替换输入源，运动层沿用已验证实现。 | **独立**：自己的 `.venv`、ROS 包、配置和 `/vision` 话题；与 `demo/` 完全独立。 | [`README.md`](Vision_Teleop/README.md) |
| [`XRoboToolkit-Teleop-Sample-Python/`](XRoboToolkit-Teleop-Sample-Python/) | XRoboToolkit 官方 Python 遥操作示例，含 MuJoCo/Placo 仿真示例。 | **独立**：官方示例工程，有自己的安装说明和 Python 包。 | [`README.md`](XRoboToolkit-Teleop-Sample-Python/README.md) |
| `XRoboToolkit-Quest-1.0.1.apk` | Quest 端 XRoboToolkit APK，供遥操作工程安装/启动使用。 | 共享的安装包文件，不是工程目录。 | — |

`.dsh/` 是开发工具配置，不参与上述工程运行，没有项目 README。

## 独立性说明

- 上述工程目录的**代码和依赖彼此独立**，各自维护 `.venv`、构建产物、配置和启动方式；不要在一个工程里 import 另一个工程的模块。
- 它们在同一台机器上运行时**共享硬件和 ROS 图**：Quest/PC-service、JAKA 控制器、AMR、相机等。不要让两个工程同时占用同一台相机，也不要同时向同一台机器人发布运动目标。
- 各工程的 `.venv`、`build/`、`install/`、`log/`、`out/`、缓存和编译产物不入库，规则见根目录 [`.gitignore`](.gitignore)。

## 安全说明

`VR_safe` 的相机/区域判定只是软件保护层，不能替代硬件急停、物理围栏和现场风险评估。真机运行前请阅读 [`VR_safe/docs/hil-checklist.md`](VR_safe/docs/hil-checklist.md)，并确认保护区、操作员站位和恢复流程。
