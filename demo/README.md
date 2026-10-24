# Quest 3 数据接收测试

此程序通过 `XRoboToolkit-PC-Service` 自带的 C++ SDK 接收 Quest 3 数据。它不依赖
ROS，也不依赖 Python pybind 包。

> Jaka 主机是 ARM64 时，必须安装 PC-service 的 `arm64` 版本。
> `amd64`/`x64` 版本中的 `libPXREARobotSDK.so` 不能在 Jaka 上链接或运行。

## 1. 启动 PC-service

在 Jaka 主机的 ARM64 Ubuntu 上安装官方 ARM64 包：

```bash
sudo apt-get update
sudo apt-get install -y build-essential
sudo dpkg -i XRoboToolkit-PC-Service_1.0.0.0_arm64.deb
sudo apt-get -f install
```

可以先检查架构和安装包：

```bash
uname -m                              # 应为 aarch64
dpkg --print-architecture             # 应为 arm64
dpkg -s roboticsservice | grep Architecture
```

如果 PC-service 尚未运行，先在一个终端启动并保持运行：

```bash
cd /opt/apps/roboticsservice
./runService.sh
```

## 2. 编译并运行接收程序

在另一个终端执行：

```bash
cd /home/carrie/my_project/XRToolKit/demo
make
./xrt_receive_test
```

`Makefile` 会根据 `uname -m` 自动查找 x86_64 或 ARM64 版本的
`libPXREARobotSDK.so`。如果 PC-service 安装在其他目录，可执行：

```bash
make XRT_SERVICE_ROOT=/PC-service安装目录
```

编译失败时先执行下面的命令，它会打印当前架构以及实际找到的 SDK 头文件和库：

```bash
make info
```

如果自动查找失败，可以在 Jaka 上定位实际文件：

```bash
find /opt/apps/roboticsservice/SDK -name PXREARobotSDK.h -print
find /opt/apps/roboticsservice/SDK -name libPXREARobotSDK.so -print
find /opt/apps/roboticsservice/SDK -name libPXREARobotSDK.so -exec file {} \;
```

`file` 输出必须是 `ARM aarch64`，不能是 `x86-64`。

默认一直运行到按 `Ctrl+C`。也可以指定采集秒数和输出文件：

```bash
# 采集 30 秒
./xrt_receive_test 30 xrt_received.jsonl
```

不同安装路径或 ARM64 主机可覆盖 SDK 路径：

```bash
make XRT_SDK_DIR=/实际库目录 XRT_SDK_INCLUDE_DIR=/实际头文件目录
```

## 3. 在 Quest 3 中发送数据

1. 打开 XRoboToolkit APK。
2. 输入 Jaka 主机的局域网 IP。
3. 按测试目的打开头显、手柄、按键或手部 Tracking 选项。
4. 点击发送，然后移动头显和手柄、按扳机/按键。

终端出现 `[DATA] First Quest state received` 即表示数据已收到。程序会把 PC-service
回调中的每条原始 `stateJson` 写入 `xrt_received.jsonl`，一行一条消息。

```bash
wc -l xrt_received.jsonl
jq . xrt_received.jsonl | head -n 80
jq -r '.functionName' xrt_received.jsonl | sort | uniq -c
```

回调结构以已安装 SDK 的 `PXREARobotSDK.h` 为准：`PXREADeviceStateJson` 的
`userData` 是 `PXREADevStateJson*`，包含设备序列号 `devID` 和变长 JSON 数据
`stateJson`。回调返回后不能继续保存 `userData` 指针，因此本示例在回调内立即复制
并写入文件。

退出码为 `0` 表示收到了至少一条状态数据；`1` 表示 SDK 初始化失败；`3` 表示程序
正常结束但没有收到 Quest 数据。
