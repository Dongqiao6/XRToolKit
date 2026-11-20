# 01: 项目壳与离线 bootstrap

**What to build:** 一个完全独立的 VR_safe 工程：自己的 Python 虚拟环境、ROS 2 工作空间、安全契约消息包、配置骨架、离线依赖/模型准备脚本和可重复的构建入口。构建后可以直接 `ros2 interface show` 安全消息，且不 import、不启动任何兄弟目录的代码。

**Blocked by:** None (can start immediately)

**Status:** implemented & verified（见 docs/status.md）

- [ ] `.venv` 不使用 `--system-site-packages`；source ROS 后 `import rclpy` 与 venv 依赖同时可用。
- [ ] `colcon build` 通过，安全契约消息/服务可被 `ros2 interface show` 看到。
- [ ] bootstrap 能准备离线 wheel、模型和构建产物；缺件时给出明确错误，不静默联网。
- [ ] 工程内没有对 `demo/`、`camera_zone/`、`Vision_Teleop/` 的 import、launch 或运行期路径依赖。
- [ ] 配置骨架存在，且启动时能识别缺失/非法配置并拒绝。
