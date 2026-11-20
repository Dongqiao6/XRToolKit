# 工位占用状态。camera_zone 发，workcell_supervisor 收。
#
# 字段是 2026-11-11 先定的一版，**等和老师对齐后可能改**。
# 枚举三值是老师定死的，下面那些附加字段是我加的——三值会把
# 「人在旁边但没进去」这档信息丢掉，监督器拿不到就没法做降速。

std_msgs/Header header

# --- 老师定的三值 ---
uint8 CLEAR=0        # 区域里没人
uint8 OCCUPIED=1     # 区域被占（人进去了，或者伸手进去了）
uint8 UNKNOWN=2      # 看不清：被遮挡、超量程、深度全空洞。按最坏处理
uint8 state

# --- 附加信息，监督器可以不理会 ---
string zone_id          # 哪个工位。单工位时固定为 "default"
bool near               # 人在区外但靠近边界，给降速用
float32 edge_m          # 离边界多远（米）。区内为正，区外为负
uint8 evidence          # 判定依据，见下面常量
string reason           # 给人看的一句话，调试和界面用
string detail_json      # 全部中间量，JSON 字符串

uint8 EV_NONE=0         # 没看到人
uint8 EV_TORSO=1        # 躯干进了区域 —— 人整个进去了
uint8 EV_LIMB=2         # 只有手/胳膊/腿伸进去
uint8 EV_HEAD=3         # 头探进去了，身子还在外面
uint8 EV_LOST=4         # 检到人但测不出三维位置
