# 感知层对保护区域的判断。它只表达“我看到了什么”，不表达“机器人能不能动”。
std_msgs/Header header

string source_id          # 例如 camera_top / force / door
string zone_id            # 例如 default / reach
uint8 CLEAR=0             # 区域里没人
uint8 NEAR=1              # 人在区外但靠近边界
uint8 OCCUPIED=2          # 区域被占（人进去或伸手）
uint8 UNKNOWN=3           # 看不清/测不出，按最坏处理
uint8 ESTOP=4             # 系统软件急停按钮（安全事件，不是占用状态）
uint8 state

float32 confidence        # 0~1，感知对本次判断的置信度；没有就填 1.0
uint8 evidence            # 与 camera_zone 的 EV_* 对齐：0 无 / 1 躯干 / 2 肢体 / 3 头 / 4 丢失
string detail_json        # 全部中间量，JSON 字符串
