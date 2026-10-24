# Quest 3 数据接收测试

此程序通过 `XRoboToolkit-PC-Service` 自带的 C++ SDK 接收 Quest 3 数据。它不依赖
ROS，也不依赖 Python pybind 包。

## 1. 启动 PC-service

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

默认一直运行到按 `Ctrl+C`。也可以指定采集秒数和输出文件：

```bash
# 采集 30 秒
./xrt_receive_test 30 xrt_received.jsonl
```

不同安装路径或 ARM64 主机可覆盖 SDK 路径：

```bash
make XRT_SDK_DIR=/actual/sdk/library/path
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
