# 用源码复制换取 VR_safe 的独立

VR_safe 要与 `demo/`、`camera_zone/` 在运行期和构建期完全隔离，而这两个目录只允许读。我们决定把需要的代码复制进 `VR_safe/` 后独立维护：不 import、不启动、不依赖兄弟目录的进程、venv 或构建产物。代价是同一份逻辑存在两个副本，后续同步只能手工进行；换取的是兄弟目录被改动、挪走或重编译都不会影响 VR_safe。

## Considered Options

- 运行期包装：VR_safe 只加安全层，按路径启动 demo 节点。改动少，但路径、venv 和已编译产物都耦合，兄弟目录一变就断。
- 直接 import 兄弟目录模块：最省代码，但违反隔离要求，且 demo 与 camera_zone 的 Python 依赖版本不同（numpy/opencv/mediapipe）。
