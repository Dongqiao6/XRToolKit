# 单臂执行状态。臂驱动发，监督器收。
# 两个布尔要分开：收到阻断（command_blocked）和真的停住了（motion_stopped）
# 不是一回事——中间隔着减速过程，演示里要证明的正是「实际停下来了」。

std_msgs/Header header

string arm              # "left" 或 "right"
bool command_blocked    # 已在驱动层阻断新指令
bool motion_stopped     # 实测已停稳
float32 tcp_speed_mps   # 末端线速度，判停稳用
string state            # 驱动自己的状态机名字
