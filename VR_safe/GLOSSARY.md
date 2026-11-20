# VR_safe

人机共存工位的安全遥操作工程：命令源产生运动目标，安全裁决器依据观察与心跳决定是否允许运动，臂控制节点执行许可。

## Language

**观察（Observation）**:
感知层对保护区域当前状态的判断，包含状态、区域标识和时间戳。
_Avoid_: 安全标记、报警、停止信号

**许可（Permit）**:
安全裁决层对机器人当前是否允许运动的结论，与观察分开。
_Avoid_: 安全状态、占用标志

**观察状态（ObservationState）**:
感知层对保护区域的四值判断：CLEAR、NEAR、OCCUPIED、UNKNOWN。
_Avoid_: 占用状态、风险等级

**许可决定（PermitDecision）**:
安全裁决器输出的三值结论：GRANTED、SLOW、REVOKED。
_Avoid_: 允许标志、速度系数

**系统软件急停（System E-Stop）**:
操作员在 Quest 上触发的全系统 L2，区别于硬件急停。
_Avoid_: 底盘急停、软急停

**停止级别（StopLevel）**:
执行层落实许可的动作：RUN、SLOW、HOLD、ESTOP。
_Avoid_: 停止、急停（HOLD 不是急停）

**观察源（SafetySource）**:
产生观察的组件，例如相机、力传感器、急停按钮。
_Avoid_: 检测器、相机节点

**执行端（SafetySink）**:
消费许可并执行停止的组件，例如臂控制、夹爪、底盘。
_Avoid_: 机器人、被控对象

**安全裁决器（SafetySupervisor）**:
把观察、心跳和人工复位合成为唯一许可的组件。
_Avoid_: 监督器、workcell_supervisor

**安全闸门（SafetyGate）**:
控制节点内部执行许可的局部逻辑：阻断新目标、持续发零增量、保持位置。
_Avoid_: 停止器、阻断器

**命令源（CommandSource）**:
产生运动目标的组件，例如 Quest 手柄桥、视觉遥操作、程序化轨迹。
_Avoid_: 遥操作、输入

**遥操作心跳（TeleopHeartbeat）**:
命令源以固定频率发出的存活信号；丢失表示整条遥操作链路不可用。
_Avoid_: 心跳包、keepalive

**离合（Clutch）**:
操作者按住才允许机械臂跟随、松开即冻结的语义。
_Avoid_: 使能、抱闸

**保护区域（GuardZone）**:
判断人体是否侵入的三维区域，可以是人工标定的多边形柱体，也可以是机械臂可达空间。
_Avoid_: 工位、操作区
