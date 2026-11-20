# 底盘执行状态。底盘节点发，supervisor / 日志收。
std_msgs/Header header

string state              # "manual" / "hold" / "estop" / "dry_run"
bool command_blocked
bool hold_active
float32 linear_mps
float32 angular_rps
string detail_json
