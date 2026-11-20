# 安全裁决器对“机器人当前是否允许运动”的唯一结论。
std_msgs/Header header

uint8 GRANTED=0           # 允许运动
uint8 SLOW=1              # 允许运动，但速度系数受限
uint8 REVOKED=2           # 不允许运动
uint8 decision

string reason             # 给人看的原因
string source             # 触发者：camera_top / teleop_lost / system_estop / confirm
bool latched              # true = 需要人工确认才能恢复
uint32 seq                # 单调递增，便于订阅端识别新消息
