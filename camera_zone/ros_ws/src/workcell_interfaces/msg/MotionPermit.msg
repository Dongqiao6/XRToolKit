# 运动许可。workcell_supervisor 发，两臂驱动收。
# allow=false 时驱动层阻断新指令并执行暂停——与指令来源无关，
# 遥操作、自主策略、固定轨迹都拦。

std_msgs/Header header

bool allow
string reason       # 为什么撤销，给操作员看
string source       # 谁撤的，例如 "camera_zone" / "operator" / "arm_fault"
