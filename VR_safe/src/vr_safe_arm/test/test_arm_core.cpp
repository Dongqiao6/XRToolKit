// vr_safe_arm 纯逻辑单元测试：不依赖 ROS、JAKA SDK 或硬件。
//
// 覆盖工单 05 的安全边界：
//   - REVOKED 阻断、强制松离合、不自动跟随；
//   - 许可恢复后必须松开再按 Grip；
//   - Grip 迟滞；
//   - 200 ms target 看门狗；
//   - SLOW 0.3 倍步长、5 mm/周期限速、y 向 ±110 mm 限位。
#include "vr_safe_arm/arm_core.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace
{
int g_failures = 0;

void expect(bool condition, const std::string & message)
{
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", message.c_str());
    ++g_failures;
  }
}

void expect_near(double actual, double expected, double tolerance, const std::string & message)
{
  if (std::fabs(actual - expected) > tolerance) {
    std::fprintf(
      stderr, "FAIL: %s（期望 %.6f，实际 %.6f）\n",
      message.c_str(), expected, actual);
    ++g_failures;
  }
}

vr_safe_arm::ArmCoreConfig base_config()
{
  vr_safe_arm::ArmCoreConfig config;
  config.slow_scale = 0.3;
  config.target_watchdog_s = 0.2;
  config.grip_press_threshold = 0.6;
  config.grip_release_threshold = 0.4;
  config.max_translation_step_mm = 5.0;
  config.translation_deadband_mm = 0.5;
  config.rotation_deadband_rad = 0.005;
  config.max_rotation_step_rad = 0.0015;
  config.stopped_speed_mps = 0.005;
  config.y_limit_mm = 110.0;
  return config;
}

// 走完“GRANTED -> 按 Grip -> 重锚定成功”的公共前置。
void enter_tracking(vr_safe_arm::ArmSafetyGate & gate, double now_s)
{
  gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "test", 1);
  const auto action = gate.update_grip(0.8, now_s);
  expect(
    action == vr_safe_arm::GripAction::kStartReArm,
    "GRANTED 后第一次按 Grip 应启动重锚定");
  gate.note_target(now_s);
  expect(gate.finish_rearm(true, now_s), "重锚定成功且仍按着时应进入跟踪");
  expect(gate.gripped(), "重锚定成功后 gripped 应为 true");
}

void test_startup_is_blocked()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  expect(gate.command_blocked(), "启动时默认 REVOKED");
  expect(!gate.tracking_allowed(), "启动时不允许跟踪");
  expect_near(gate.step_scale(), 0.0, 1e-12, "REVOKED 时步长缩放应为 0");
  expect(
    gate.state_name(false, true) == "blocked",
    "启动时状态应为 blocked");
  expect(
    gate.state_name(true, true) == "dry_run/blocked",
    "dry-run 状态应带 dry_run/ 前缀");
  expect(
    gate.state_name(false, false) == "disconnected",
    "驱动未就绪时状态应为 disconnected");
}

void test_permit_revoked_blocks_and_forces_regrip()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  enter_tracking(gate, 1.0);
  expect(
    gate.state_name(false, true) == "tracking",
    "跟踪状态应为 tracking");

  // 操作者仍按着 Grip 时许可被撤销。
  gate.update_permit(vr_safe_arm::PermitDecision::kRevoked, "camera occupied", 2);
  expect(gate.command_blocked(), "REVOKED 后应阻断新 target");
  expect(!gate.gripped(), "REVOKED 后必须强制松离合");
  expect(gate.regrip_required(), "REVOKED 后要求重新按 Grip");
  expect_near(gate.step_scale(), 0.0, 1e-12, "REVOKED 时不允许步长");

  // 许可恢复但操作者还按着旧的那次 Grip：不能自动跟随。
  gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "confirm", 3);
  expect(!gate.command_blocked(), "恢复后解除阻断");
  expect(!gate.gripped(), "恢复后不得自动跟随");
  expect(
    gate.update_grip(0.8, 2.0) == vr_safe_arm::GripAction::kWaitForRelease,
    "操作者还按着时必须等松开");

  // 松开 -> 再按，才允许重新锚定。
  expect(
    gate.update_grip(0.0, 2.1) == vr_safe_arm::GripAction::kNone,
    "松开本身不产生动作");
  expect(!gate.regrip_required(), "松开后清除重新按门闩");
  expect(
    gate.update_grip(0.8, 2.2) == vr_safe_arm::GripAction::kStartReArm,
    "重新按下应启动重锚定");
  expect(gate.finish_rearm(true, 2.2), "重新锚定后恢复跟踪");
}

void test_revoked_without_press_needs_only_one_new_press()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "test", 1);
  // 操作者没按 Grip 时许可被撤销：不应留下“先松开”门闩。
  gate.update_permit(vr_safe_arm::PermitDecision::kRevoked, "occupied", 2);
  expect(!gate.regrip_required(), "未按 Grip 时 REVOKED 不应要求先松开");
  gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "confirm", 3);
  expect(
    gate.update_grip(0.8, 2.0) == vr_safe_arm::GripAction::kStartReArm,
    "未按 Grip 时经历 REVOKED 后，第一次新按下就应启动重锚定");
  expect(gate.finish_rearm(true, 2.0), "第一次新按下应能进入跟踪");
}

void test_grip_hysteresis()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  enter_tracking(gate, 1.0);

  // 0.5 在迟滞带内：因为之前按着，仍算按着，不应松离合。
  expect(
    gate.update_grip(0.5, 1.1) == vr_safe_arm::GripAction::kNone,
    "迟滞带内的值不应改变按压状态");
  expect(gate.gripped(), "迟滞带内应保持跟踪");

  // 低于下沿才松开。
  expect(
    gate.update_grip(0.39, 1.2) == vr_safe_arm::GripAction::kReleaseAndHold,
    "低于释放阈值应松离合");
  expect(!gate.gripped(), "松开后不应跟踪");

  // 新按下必须超过上沿；0.55 不应触发。
  expect(
    gate.update_grip(0.55, 1.3) == vr_safe_arm::GripAction::kNone,
    "未超过按下阈值不应启动重锚定");
  expect(
    gate.update_grip(0.61, 1.4) == vr_safe_arm::GripAction::kStartReArm,
    "超过按下阈值应启动重锚定");
}

void test_target_watchdog()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  enter_tracking(gate, 10.0);
  gate.note_target(10.0);

  expect(!gate.check_target_watchdog(10.199), "199 ms 内不应触发看门狗");
  expect(gate.gripped(), "199 ms 内仍应跟踪");

  expect(gate.check_target_watchdog(10.201), "超过 200 ms 应触发看门狗");
  expect(!gate.gripped(), "看门狗触发后应松离合");
  expect(gate.regrip_required(), "看门狗触发后要求重新按 Grip");
  expect(
    gate.state_name(false, true) == "hold",
    "看门狗触发后状态应为 hold");

  // 还按着不能再自动恢复，必须先松开。
  expect(
    gate.update_grip(0.8, 10.3) == vr_safe_arm::GripAction::kWaitForRelease,
    "看门狗触发后仍按着 Grip 不得自动重锚定");
  expect(
    gate.update_grip(0.0, 10.4) == vr_safe_arm::GripAction::kNone,
    "松开清除门闩");
  expect(
    gate.update_grip(0.8, 10.5) == vr_safe_arm::GripAction::kStartReArm,
    "重新按下才允许再次跟踪");
}

void test_slow_scaling_and_cycle_limit()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "clear", 1);
  expect_near(gate.step_scale(), 1.0, 1e-12, "GRANTED 全速");

  // 5 mm/周期限速，保持方向。
  vr_safe_arm::Vec3 fast;
  fast.x = 10.0;
  fast.y = -20.0;
  fast.z = 0.0;
  const auto granted_step = gate.limit_translation_step(fast);
  expect_near(granted_step.x, 2.5, 1e-9, "GRANTED 时 5 mm 限速 x");
  expect_near(granted_step.y, -5.0, 1e-9, "GRANTED 时 5 mm 限速 y");
  expect_near(
    gate.limit_rotation_step(0.01), 0.0015, 1e-12,
    "GRANTED 时姿态 0.0015 rad/周期限速");

  gate.update_permit(vr_safe_arm::PermitDecision::kSlow, "near", 2);
  expect_near(gate.step_scale(), 0.3, 1e-12, "SLOW 缩放 0.3");
  const auto slow_step = gate.limit_translation_step(fast);
  expect_near(slow_step.x, 0.75, 1e-9, "SLOW 时平移步长按 0.3 缩放 x");
  expect_near(slow_step.y, -1.5, 1e-9, "SLOW 时平移步长按 0.3 缩放 y");
  expect_near(
    gate.limit_rotation_step(0.01), 0.00045, 1e-12,
    "SLOW 时姿态步长按 0.3 缩放");
  expect_near(
    gate.translation_step_cap_mm(), 1.5, 1e-9, "SLOW 平移上限 1.5 mm/周期");
  expect_near(
    gate.rotation_step_cap_rad(), 0.00045, 1e-12, "SLOW 姿态上限");

  // 小位移不受限速影响，但仍按 SLOW 缩放。
  vr_safe_arm::Vec3 small;
  small.x = 2.0;
  const auto small_step = gate.limit_translation_step(small);
  expect_near(small_step.x, 0.6, 1e-9, "SLOW 下小位移同样缩放 0.3");

  // 回到 GRANTED 不锁存，自动全速（但不会自动跟随，操作者已处于 tracking）。
  gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "clear", 3);
  expect_near(gate.step_scale(), 1.0, 1e-12, "GRANTED 恢复全速");
}

void test_y_limit()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  // 左臂：TCP y 不得小于 -110。
  expect_near(
    gate.apply_workspace_y_limit(-200.0, 0.0, false, true, 0.0),
    -110.0, 1e-9, "左臂 y 下限 -110 mm");
  expect_near(
    gate.apply_workspace_y_limit(-50.0, 0.0, false, true, 0.0),
    -50.0, 1e-9, "左臂限位内不修改");
  // 右臂：TCP y 不得大于 +110。
  expect_near(
    gate.apply_workspace_y_limit(200.0, 0.0, true, true, 0.0),
    110.0, 1e-9, "右臂 y 上限 +110 mm");
  // 按下 Grip 时 TCP 已经在边界附近：只允许剩余余量。
  expect_near(
    gate.apply_workspace_y_limit(100.0, 0.0, true, true, 100.0),
    10.0, 1e-9, "右臂按绝对 TCP y 计算剩余余量");
  // 没读到 TCP 时不做限位，保持基线行为。
  expect_near(
    gate.apply_workspace_y_limit(200.0, 0.0, true, false, 0.0),
    200.0, 1e-9, "TCP 无效时不限位");
}

void test_motion_stopped_and_state_names()
{
  vr_safe_arm::ArmSafetyGate gate(base_config());
  expect(gate.motion_stopped(0.001, true, false), "低于 5 mm/s 应判停稳");
  expect(!gate.motion_stopped(0.05, true, false), "高于 5 mm/s 不应判停稳");
  expect(gate.motion_stopped(0.0, false, true), "速度无效但 hold 流在发应判停稳");
  expect(!gate.motion_stopped(0.0, false, false), "速度无效且无 hold 流不应判停稳");

  gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "clear", 1);
  expect(
    gate.state_name(true, true) == "dry_run/hold",
    "未按 Grip 时 dry-run 状态为 hold");
  gate.update_grip(0.8, 1.0);
  gate.finish_rearm(true, 1.0);
  gate.update_permit(vr_safe_arm::PermitDecision::kSlow, "near", 2);
  expect(
    gate.state_name(true, true) == "dry_run/tracking_slow",
    "SLOW 跟踪状态应为 tracking_slow");
}

void test_config_sanitizing()
{
  vr_safe_arm::ArmCoreConfig config = base_config();
  config.slow_scale = 2.0;
  config.grip_press_threshold = 0.3;
  config.grip_release_threshold = 0.5;
  config.target_watchdog_s = -1.0;
  config.max_translation_step_mm = 0.0;
  vr_safe_arm::ArmSafetyGate gate(config);
  gate.update_permit(vr_safe_arm::PermitDecision::kSlow, "near", 1);
  expect_near(gate.step_scale(), 1.0, 1e-12, "slow_scale 上限收敛到 1.0");
  expect_near(
    gate.translation_step_cap_mm(), 5.0, 1e-9,
    "非法限速退回 5 mm/周期");
  // 迟滞阈值非法时退回 0.6/0.4：0.5 不会触发按下。
  expect(
    gate.update_grip(0.5, 1.0) == vr_safe_arm::GripAction::kNone,
    "非法迟滞阈值退回默认值");

  // 看门狗时间非法时退回 200 ms。
  vr_safe_arm::ArmSafetyGate watchdog_gate(config);
  watchdog_gate.update_permit(vr_safe_arm::PermitDecision::kGranted, "clear", 1);
  watchdog_gate.update_grip(0.8, 1.0);
  watchdog_gate.note_target(1.0);
  expect(watchdog_gate.finish_rearm(true, 1.0), "前置跟踪应建立");
  expect(
    !watchdog_gate.check_target_watchdog(1.15),
    "非法看门狗参数应退回 200 ms");
  expect(
    watchdog_gate.check_target_watchdog(1.25),
    "退回 200 ms 后 250 ms 应触发");
}
}  // namespace

int main()
{
  test_startup_is_blocked();
  test_permit_revoked_blocks_and_forces_regrip();
  test_revoked_without_press_needs_only_one_new_press();
  test_grip_hysteresis();
  test_target_watchdog();
  test_slow_scaling_and_cycle_limit();
  test_y_limit();
  test_motion_stopped_and_state_names();
  test_config_sanitizing();

  if (g_failures != 0) {
    std::fprintf(stderr, "%d 项检查失败\n", g_failures);
    return EXIT_FAILURE;
  }
  std::printf("PASS: arm_core 全部安全边界检查通过\n");
  return EXIT_SUCCESS;
}
