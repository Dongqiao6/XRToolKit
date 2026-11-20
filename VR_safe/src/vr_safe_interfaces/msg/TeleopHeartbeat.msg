# 命令源存活心跳。supervisor 收不到就把整条遥操作链路当成失联。
std_msgs/Header header

string source_id          # 例如 quest
uint32 seq
