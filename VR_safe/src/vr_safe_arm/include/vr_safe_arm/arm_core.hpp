// 臂执行端的纯逻辑核心：许可门、离合状态、本地 target 看门狗、步长限幅。
//
// 这里故意不依赖 ROS 和 JAKA SDK：
//   - 单元测试可以在没有驱动、没有硬件的情况下覆盖安全边界；
//   - 控制节点的回调只负责把消息翻译成这里的输入，再按输出动作操作 SDK。
//
// 所有时间参数统一用“秒”，由调用方注入（节点用 steady_clock，测试用假时钟），
// 这样看门狗和迟滞可以确定性地测试，不依赖真实睡眠。
#ifndef VR_SAFE_ARM_ARM_CORE_HPP_
#define VR_SAFE_ARM_ARM_CORE_HPP_

#include <cstdint>
#include <mutex>
#include <string>

namespace vr_safe_arm
{

// 与 vr_safe_interfaces/MotionPermit 的 GRANTED/SLOW/REVOKED 数值一致。
enum class PermitDecision : uint8_t
{
  kGranted = 0,
  kSlow = 1,
  kRevoked = 2,
};

const char * permit_decision_name(PermitDecision decision);

// 单个臂的安全参数。默认值来自 docs/design.md 与 vision_rc 基线。
struct ArmCoreConfig
{
  double slow_scale = 0.3;                 // SLOW 时平移/姿态步长缩放系数
  double target_watchdog_s = 0.2;          // gripped 时多久没有新 target 判跟踪丢失
  double grip_press_threshold = 0.6;       // Grip 按下阈值（迟滞上沿）
  double grip_release_threshold = 0.4;     // Grip 松开阈值（迟滞下沿）
  double max_translation_step_mm = 5.0;    // 每周期平移限速（保留基线 5 mm/周期）
  double translation_deadband_mm = 0.5;    // 小于该值的平移视为手柄噪声
  double rotation_deadband_rad = 0.005;    // 小于该值的姿态偏差不动作
  double max_rotation_step_rad = 0.0015;   // 每周期姿态限速（保留基线）
  double stopped_speed_mps = 0.005;        // 判定“已停稳”的末端线速度阈值
  double y_limit_mm = 110.0;               // 双臂共享中心线限位
};

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// Grip 输入处理结果。节点据此决定是否启动后台重锚定、是否开 hold stream。
enum class GripAction
{
  kNone = 0,           // 无事发生（还按着、正在重锚定等）
  kStartReArm = 1,     // 新的按下沿：启动后台重锚定
  kReleaseAndHold = 2, // 松开离合：立即冻结并保持零增量
  kWaitForRelease = 3, // 许可刚恢复但操作者还按着：必须松开后重新按
};

// 臂的许可执行状态机。
//
// 线程模型：后台重锚定线程会调用 finish_rearm()，ROS 执行器线程会调用其余方法，
// 因此所有公开方法都加互斥锁。临界区都很短，不涉及 SDK 调用。
class ArmSafetyGate
{
public:
  explicit ArmSafetyGate(const ArmCoreConfig & config = ArmCoreConfig());

  const ArmCoreConfig & config() const { return config_; }

  // ── 许可 ──
  // 收到新的 MotionPermit。REVOKED 会立即强制松离合并置“必须重新按 Grip”。
  // 从 REVOKED 恢复时绝不自动跟随，只解除阻断，等操作者重新按 Grip。
  void update_permit(
    PermitDecision decision, const std::string & reason, uint32_t seq);
  PermitDecision permit() const;
  std::string permit_reason() const;
  uint32_t permit_seq() const;
  bool command_blocked() const;   // true = REVOKED，阻断新 target
  bool tracking_allowed() const;  // true = GRANTED/SLOW
  double step_scale() const;      // GRANTED=1.0；SLOW=slow_scale；REVOKED=0.0

  // ── 离合 ──
  // 输入本臂的 Grip 轴值，返回节点要执行的动作。内部维护迟滞和重新按 Grip 门闩。
  GripAction update_grip(double grip_value, double now_s);
  // 强制松离合（REVOKED、看门狗、驱动失败时调用）。require_regrip=true 时，
  // 即使操作者仍按着 Grip 也必须先松开再按，才允许重新锚定。
  void force_release(bool require_regrip);
  // 后台重锚定结束。只有成功、仍按着 Grip、许可允许且没有被要求重新按 Grip 时
  // 才进入跟踪；返回 true 表示已开始跟踪（gripped）。
  bool finish_rearm(bool success, double now_s);
  bool grip_pressed() const;
  bool gripped() const;
  bool rearming() const;
  bool regrip_required() const;

  // ── target 看门狗 ──
  // 每收到一帧 target 调用；时间段内没有新 target 时，看门狗在 check 时触发。
  void note_target(double now_s);
  // 返回 true 表示本次调用触发了看门狗：已强制松离合并要求重新按 Grip。
  bool check_target_watchdog(double now_s);
  double last_target_time_s() const;

  // ── 步长与限位 ──
  // 先按 slow_scale 缩放，再按 5 mm/周期限幅（保持方向）。
  Vec3 limit_translation_step(const Vec3 & raw_delta) const;
  // 先按每周期姿态上限限幅，再按 slow_scale 缩放。
  double limit_rotation_step(double requested_step_rad) const;
  double translation_step_cap_mm() const;
  double rotation_step_cap_rad() const;

  // 工作空间 y 向单侧限位：左臂 y > -110、右臂 y < +110。
  // ctrl_y/origin_y 是手柄目标（mm），tcp_y_at_grip 是按下 Grip 时的实际 TCP y。
  double apply_workspace_y_limit(
    double ctrl_y, double origin_y, bool arm_is_right,
    bool tcp_valid, double tcp_y_at_grip) const;

  // 末端速度判停。速度无效时用 hold stream 是否在持续发零增量兜底。
  bool motion_stopped(
    double speed_mps, bool speed_valid, bool hold_stream_active) const;

  // 驱动自己的状态机名字。dry_run 加 "dry_run/" 前缀，便于现场一眼区分真假节点。
  std::string state_name(bool dry_run, bool driver_ready) const;

private:
  double step_scale_locked() const;
  void force_release_locked(bool require_regrip);

  ArmCoreConfig config_;
  mutable std::mutex mutex_;

  PermitDecision permit_ = PermitDecision::kRevoked;
  std::string permit_reason_ = "启动中，等待许可";
  uint32_t permit_seq_ = 0;

  bool grip_pressed_ = false;
  bool gripped_ = false;
  bool rearming_ = false;
  bool regrip_required_ = false;
  double last_target_time_s_ = 0.0;
};

}  // namespace vr_safe_arm

#endif  // VR_SAFE_ARM_ARM_CORE_HPP_
