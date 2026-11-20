# 单臂执行状态。臂控制节点发，supervisor / 日志收。
std_msgs/Header header

string arm                # "left" / "right"
bool command_blocked      # 已阻断新指令
bool motion_stopped       # 实测已停稳
float32 tcp_speed_mps     # 末端线速度，判停稳用
string state              # 驱动自己的状态机名字
