#include "vr_safe_arm/arm_core.hpp"

#include <algorithm>
#include <cmath>

namespace vr_safe_arm
{
namespace
{
constexpr double kDefaultSlowScale = 0.3;
constexpr double kDefaultWatchdogS = 0.2;
constexpr double kDefaultPressThreshold = 0.6;
constexpr double kDefaultReleaseThreshold = 0.4;
constexpr double kDefaultMaxTranslationStepMm = 5.0;
constexpr double kDefaultTranslationDeadbandMm = 0.5;
constexpr double kDefaultRotationDeadbandRad = 0.005;
constexpr double kDefaultMaxRotationStepRad = 0.0015;
constexpr double kDefaultStoppedSpeedMps = 0.005;
constexpr double kDefaultYLimitMm = 110.0;

double finite_or(double value, double fallback)
{
  return std::isfinite(value) ? value : fallback;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

// 所有参数都在构造时做一次合法性收敛：现场调参写错时退回安全默认值，
// 而不是让 NaN/负数进入控制回路。
ArmCoreConfig sanitize(const ArmCoreConfig & input)
{
  ArmCoreConfig cfg;
  cfg.slow_scale = clamp(finite_or(input.slow_scale, kDefaultSlowScale), 0.0, 1.0);
  cfg.target_watchdog_s = finite_or(input.target_watchdog_s, kDefaultWatchdogS);
  if (cfg.target_watchdog_s <= 0.0) {
    cfg.target_watchdog_s = kDefaultWatchdogS;
  }
  cfg.grip_press_threshold = clamp(
    finite_or(input.grip_press_threshold, kDefaultPressThreshold), 0.0, 1.0);
  cfg.grip_release_threshold = clamp(
    finite_or(input.grip_release_threshold, kDefaultReleaseThreshold), 0.0, 1.0);
  if (cfg.grip_press_threshold <= cfg.grip_release_threshold) {
    cfg.grip_press_threshold = kDefaultPressThreshold;
    cfg.grip_release_threshold = kDefaultReleaseThreshold;
  }
  cfg.max_translation_step_mm = finite_or(
    input.max_translation_step_mm, kDefaultMaxTranslationStepMm);
  if (cfg.max_translation_step_mm <= 0.0) {
    cfg.max_translation_step_mm = kDefaultMaxTranslationStepMm;
  }
  cfg.translation_deadband_mm = finite_or(
    input.translation_deadband_mm, kDefaultTranslationDeadbandMm);
  if (cfg.translation_deadband_mm < 0.0) {
    cfg.translation_deadband_mm = kDefaultTranslationDeadbandMm;
  }
  cfg.rotation_deadband_rad = finite_or(
    input.rotation_deadband_rad, kDefaultRotationDeadbandRad);
  if (cfg.rotation_deadband_rad < 0.0) {
    cfg.rotation_deadband_rad = kDefaultRotationDeadbandRad;
  }
  cfg.max_rotation_step_rad = finite_or(
    input.max_rotation_step_rad, kDefaultMaxRotationStepRad);
  if (cfg.max_rotation_step_rad <= 0.0) {
    cfg.max_rotation_step_rad = kDefaultMaxRotationStepRad;
  }
  cfg.stopped_speed_mps = finite_or(input.stopped_speed_mps, kDefaultStoppedSpeedMps);
  if (cfg.stopped_speed_mps < 0.0) {
    cfg.stopped_speed_mps = kDefaultStoppedSpeedMps;
  }
  cfg.y_limit_mm = finite_or(input.y_limit_mm, kDefaultYLimitMm);
  if (cfg.y_limit_mm <= 0.0) {
    cfg.y_limit_mm = kDefaultYLimitMm;
  }
  return cfg;
}
}  // namespace

const char * permit_decision_name(PermitDecision decision)
{
  switch (decision) {
    case PermitDecision::kGranted:
      return "GRANTED";
    case PermitDecision::kSlow:
      return "SLOW";
    case PermitDecision::kRevoked:
      return "REVOKED";
    default:
      return "UNKNOWN";
  }
}

ArmSafetyGate::ArmSafetyGate(const ArmCoreConfig & config)
: config_(sanitize(config))
{
}

void ArmSafetyGate::update_permit(
  PermitDecision decision, const std::string & reason, uint32_t seq)
{
  std::lock_guard<std::mutex> lock(mutex_);
  permit_ = decision;
  permit_reason_ = reason;
  permit_seq_ = seq;
  if (decision == PermitDecision::kRevoked) {
    // 阻断新指令、强制松离合。只有 REVOKED 到达时操作者仍按着 Grip，才需要
    // “先松开再按”的门闩：否则残留的按压值会在许可恢复后直接恢复跟随。
    // 如果此刻根本没按 Grip，下一次按下本身就是新离合，不应被门闩吃掉。
    const bool press_held = grip_pressed_;
    force_release_locked(false);
    regrip_required_ = press_held;
  }
}

PermitDecision ArmSafetyGate::permit() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return permit_;
}

std::string ArmSafetyGate::permit_reason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return permit_reason_;
}

uint32_t ArmSafetyGate::permit_seq() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return permit_seq_;
}

bool ArmSafetyGate::command_blocked() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return permit_ == PermitDecision::kRevoked;
}

bool ArmSafetyGate::tracking_allowed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return permit_ != PermitDecision::kRevoked;
}

double ArmSafetyGate::step_scale() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return step_scale_locked();
}

double ArmSafetyGate::step_scale_locked() const
{
  switch (permit_) {
    case PermitDecision::kGranted:
      return 1.0;
    case PermitDecision::kSlow:
      return config_.slow_scale;
    case PermitDecision::kRevoked:
    default:
      return 0.0;
  }
}

GripAction ArmSafetyGate::update_grip(double grip_value, double now_s)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Schmitt 触发：按下要超过上沿，松开要低于下沿，避免 Grip 在 0.5 附近抖动。
  const bool was_pressed = grip_pressed_;
  grip_pressed_ = was_pressed ?
    (grip_value > config_.grip_release_threshold) :
    (grip_value > config_.grip_press_threshold);

  if (!grip_pressed_) {
    const bool was_gripped = gripped_;
    gripped_ = false;
    // 已松开即视为完成“重新按 Grip”的前半段；下一次按下沿才是真正的新离合。
    regrip_required_ = false;
    return was_gripped ? GripAction::kReleaseAndHold : GripAction::kNone;
  }

  if (permit_ == PermitDecision::kRevoked) {
    // 许可阻断时按 Grip 不产生任何运动，并记住这次按压不能算恢复后的新离合。
    gripped_ = false;
    regrip_required_ = true;
    return GripAction::kWaitForRelease;
  }

  if (regrip_required_) {
    // 许可刚恢复，操作者还按着旧的那一次 Grip：等松开。
    return GripAction::kWaitForRelease;
  }

  if (gripped_ || rearming_) {
    return GripAction::kNone;
  }

  // 新的按下沿：由节点启动后台重锚定，成功后才进入跟踪。
  rearming_ = true;
  last_target_time_s_ = now_s;
  return GripAction::kStartReArm;
}

void ArmSafetyGate::force_release(bool require_regrip)
{
  std::lock_guard<std::mutex> lock(mutex_);
  force_release_locked(require_regrip);
}

void ArmSafetyGate::force_release_locked(bool require_regrip)
{
  gripped_ = false;
  if (require_regrip) {
    regrip_required_ = true;
  }
}

bool ArmSafetyGate::finish_rearm(bool success, double now_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rearming_ = false;
  if (!success) {
    gripped_ = false;
    return false;
  }
  // 重锚定期间可能发生：许可被撤销、操作者松开 Grip、或看门狗要求重新按。
  // 任一条件不满足都不能进入跟踪，宁可继续 HOLD。
  if (!grip_pressed_ || permit_ == PermitDecision::kRevoked || regrip_required_) {
    gripped_ = false;
    return false;
  }
  gripped_ = true;
  last_target_time_s_ = now_s;
  return true;
}

bool ArmSafetyGate::grip_pressed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return grip_pressed_;
}

bool ArmSafetyGate::gripped() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return gripped_;
}

bool ArmSafetyGate::rearming() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rearming_;
}

bool ArmSafetyGate::regrip_required() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return regrip_required_;
}

void ArmSafetyGate::note_target(double now_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_target_time_s_ = now_s;
}

bool ArmSafetyGate::check_target_watchdog(double now_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gripped_) {
    return false;
  }
  const double elapsed = now_s - last_target_time_s_;
  if (elapsed <= config_.target_watchdog_s) {
    return false;
  }
  // 跟踪链路失联：松离合、开 hold，并要求重新按 Grip 才能再跟随。
  gripped_ = false;
  regrip_required_ = true;
  return true;
}

double ArmSafetyGate::last_target_time_s() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_target_time_s_;
}

Vec3 ArmSafetyGate::limit_translation_step(const Vec3 & raw_delta) const
{
  double scale = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scale = step_scale_locked();
  }

  // 先按基线的 5 mm/周期限速（保持方向），再乘许可缩放。
  // 顺序不能反：反过来时大位移会先被缩放、再被 5 mm 上限截断，
  // SLOW 的实际每周期上限仍可能是 5 mm，起不到降速作用。
  Vec3 capped = raw_delta;
  const double raw_max_abs = std::max(
    std::fabs(raw_delta.x), std::max(std::fabs(raw_delta.y), std::fabs(raw_delta.z)));
  if (raw_max_abs > config_.max_translation_step_mm) {
    const double limit = config_.max_translation_step_mm / raw_max_abs;
    capped.x *= limit;
    capped.y *= limit;
    capped.z *= limit;
  }

  Vec3 out;
  out.x = capped.x * scale;
  out.y = capped.y * scale;
  out.z = capped.z * scale;
  return out;
}

double ArmSafetyGate::limit_rotation_step(double requested_step_rad) const
{
  if (!std::isfinite(requested_step_rad) || requested_step_rad <= 0.0) {
    return 0.0;
  }
  double scale = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scale = step_scale_locked();
  }
  const double capped = std::min(requested_step_rad, config_.max_rotation_step_rad);
  return capped * scale;
}

double ArmSafetyGate::translation_step_cap_mm() const
{
  return config_.max_translation_step_mm * step_scale();
}

double ArmSafetyGate::rotation_step_cap_rad() const
{
  return config_.max_rotation_step_rad * step_scale();
}

double ArmSafetyGate::apply_workspace_y_limit(
  double ctrl_y, double origin_y, bool arm_is_right,
  bool tcp_valid, double tcp_y_at_grip) const
{
  if (!tcp_valid) {
    return ctrl_y;
  }
  // 预测的绝对 y = 按 Grip 时的 TCP y + 手柄自按下以来的 y 位移。
  const double predicted_y = tcp_y_at_grip + (ctrl_y - origin_y);
  if (arm_is_right && predicted_y > config_.y_limit_mm) {
    return origin_y + (config_.y_limit_mm - tcp_y_at_grip);
  }
  if (!arm_is_right && predicted_y < -config_.y_limit_mm) {
    return origin_y + (-config_.y_limit_mm - tcp_y_at_grip);
  }
  return ctrl_y;
}

bool ArmSafetyGate::motion_stopped(
  double speed_mps, bool speed_valid, bool hold_stream_active) const
{
  if (speed_valid) {
    return std::fabs(speed_mps) < config_.stopped_speed_mps;
  }
  // 没有实测速度时，只有“已松离合且零增量流在持续发”才敢报停稳。
  return hold_stream_active;
}

std::string ArmSafetyGate::state_name(bool dry_run, bool driver_ready) const
{
  std::string name;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!driver_ready) {
      name = "disconnected";
    } else if (permit_ == PermitDecision::kRevoked) {
      name = "blocked";
    } else if (rearming_) {
      name = "arming";
    } else if (gripped_) {
      name = (permit_ == PermitDecision::kSlow) ? "tracking_slow" : "tracking";
    } else {
      name = "hold";
    }
  }
  return dry_run ? ("dry_run/" + name) : name;
}

}  // namespace vr_safe_arm
