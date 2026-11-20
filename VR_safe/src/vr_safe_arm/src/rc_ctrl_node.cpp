// vr_safe_arm 单臂控制节点（工单 05，基线 Vision_Teleop/src/vision_rc）。
//
// JAKA SDK 每进程只允许一条连接（libjakaAPI.so 的全局 robot 句柄），因此本节点
// 只驱动一台臂；launch 必须起两个进程，左右臂各一个。禁止在一个进程里连两台臂。
//
// 相对基线的改动：
//   1. 订阅 /vr_safe/permit（RELIABLE + TRANSIENT_LOCAL），REVOKED 时阻断新 target、
//      强制松离合、持续发零增量、保持伺服使能；恢复后必须重新按 Grip 并重新锚定。
//   2. SLOW 时保留跟随，但平移/姿态步长按 slow_scale 缩放。
//   3. 本地 target 看门狗：gripped 时 200 ms 没有新 target，按跟踪丢失处理。
//   4. dry_run 参数：跳过 JAKA SDK 的 login/power/enable/servo/gripper 调用，
//      但仍走同一套许可/离合/看门狗逻辑并发布 ArmState，供无硬件验证。
//   5. 以 10 Hz 发布 ArmState 到 /vr_safe/arm/{left,right}/state。
//
// 保留基线行为：Grip 迟滞、5 mm/周期限速、y 向 ±110 mm 限位、碰撞检测、servo_hold。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <clocale>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "vr_safe_interfaces/msg/arm_state.hpp"
#include "vr_safe_interfaces/msg/motion_permit.hpp"

#include "vr_safe_arm/arm_core.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/orientation_math.hpp"
#include "jaka_driver/pgi140_gripper.hpp"

using namespace std;

// SDK 返回码 -> 可读信息，便于日志排查。
map<int, string> mapErr = {
    {2,   "ERR_FUCTION_CALL_ERROR"},
    {-1,  "ERR_INVALID_HANDLER"},
    {-2,  "ERR_INVALID_PARAMETER"},
    {-3,  "ERR_COMMUNICATION_ERR"},
    {-4,  "ERR_KINE_INVERSE_ERR"},
    {-5,  "ERR_EMERGENCY_PRESSED"},
    {-6,  "ERR_NOT_POWERED"},
    {-7,  "ERR_NOT_ENABLED"},
    {-8,  "ERR_DISABLE_SERVOMODE"},
    {-9,  "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"}
};

// JAKA SDK 的 robot 句柄：一个进程只能有一条连接。
JAKAZuRobot robot;

// 控制器位姿：平移（mm）+ 四元数。四元数全程保留，只有最后算 servo_p 增量时才
// 由 SDK 转成 RPY。
struct CtrlPose {
    CartesianTran tran;   // mm
    Quaternion   quat;    // s == w（SDK 的标量对应 ROS 的 w）
};

// ── 离合与重锚定 ──
// 后台重锚定线程与目标回调之间用 rearming 做互斥：重锚定期间目标回调不碰 SDK。
// gripped 的权威状态在 gate 里，这里只保留 rearming 给 SDK 串行化用。
std::atomic<bool> rearming{false};
std::atomic<bool> hold_stream_enabled{false};
// supervisor 死了以后不能再拿最后一条 GRANTED 一直动。
std::atomic<int64_t> last_permit_ms{0};
double permit_watchdog_ms = 300.0;
int hold_stream_failure_count = 0;
std::thread rearm_thread;
bool debug = false;

// 重锚定时捕获的手柄原点 / 上一周期手柄位姿 / 已经命令过的累计旋转。
CtrlPose ctrl_origin;
CtrlPose ctrl_prev;
Quaternion commanded_rotation_delta = jaka_teleop::quat_identity();

// 最新手柄目标：目标回调写，重锚定线程在按 Grip 时读，因此用互斥锁保护。
geometry_msgs::msg::PoseStamped latest_pose;
std::mutex latest_mtx;

// 臂连接状态与身份。
bool arm_ok = false;
bool dry_run = false;
std::string arm_name = "left";
int grip_axis = 0;           // Joy.axes 下标：0 = 左 Grip，1 = 右 Grip
int gripper_axis = 2;        // 扳机轴：2 = 左扳机，3 = 右扳机
bool gripper_use_button = false;
int gripper_button_index = 0;
bool gripper_linear = true;  // true = 模拟量扳机，false = 数字按键
double gripper_trigger_deadband = 0.02;  // 触发新 Grip() 的最小变化
double prev_gripper_trigger = 0.0;
// 夹爪锁定：左手柄 X / 右手柄 A 翻转“冻结当前开合”。
bool gripper_locked = false;
bool prev_lock_button = false;
int gripper_lock_button_index = 0;
std::unique_ptr<pgi140::Pgi140Gripper> gripper;

// 姿态跟随参数（与基线一致）。
bool use_rpy_ctrl = true;
double rotation_scale = 0.5;
double rotation_gain = 0.5;
double rotation_deadband = 0.005;   // rad
double rotation_max_step = 0.0015;  // rad / 8 ms 周期
int rotation_sign = 1;
double grip_press_threshold = 0.6;
double grip_release_threshold = 0.4;
bool enable_y_limit = true;
bool arm_is_right = false;
double arm_tcp_y_at_grip = 0.0;     // 按 Grip 时的绝对 TCP y（mm）
bool arm_tcp_valid = false;         // 按 Grip 时 TCP 读取是否成功
CartesianPose arm_tcp_at_grip_pose{};  // 按 Grip 时的完整 TCP 位姿（mm）

// ── 安全闸门 & ArmState ──
// gate 是许可/离合/看门狗的权威状态机；ROS 回调只做翻译和 SDK 动作。
// 用 unique_ptr 是因为它要在 main 里读完参数后才构造。
std::unique_ptr<vr_safe_arm::ArmSafetyGate> gate;
std::shared_ptr<rclcpp::Node> g_node;
rclcpp::Publisher<vr_safe_interfaces::msg::ArmState>::SharedPtr arm_state_pub;

// dry_run 下模拟的 TCP 位置（mm）。初值给一个典型工作点，便于测 y 限位。
double sim_tcp_x = 400.0;
double sim_tcp_y = 0.0;
double sim_tcp_z = 200.0;

// 10 Hz 状态发布用的速度采样。实测优先；读不到时用 hold stream 兜底。
double last_tcp_sample_t = 0.0;
double last_tcp_sample_x = 0.0;
double last_tcp_sample_y = 0.0;
double last_tcp_sample_z = 0.0;
bool has_tcp_sample = false;
double tcp_speed_mps = 0.0;
double tcp_speed_stamp = 0.0;
bool tcp_speed_valid = false;

// 轻量黑匣子：每次 servo_p 都记录指令增量、命令点、采样 TCP、关节角、错误码和
// SDK 返回值。只做观测：打开或写文件永远不会改变控制路径。
std::ofstream blackbox;
double blackbox_flush_time = 0.0;
CartesianPose actual_tcp{};
JointValue actual_joints{};
bool feedback_valid = false;
double last_feedback_us = 0.0;
int last_errcode = 0;
int feedback_source = 0;
bool feedback_polling = false;
int feedback_stride = 1;
unsigned long feedback_tick = 0;
bool status_polling = false;
int status_stride = 13;
unsigned long status_tick = 0;
double last_cmd_x = 0.0;
double last_cmd_y = 0.0;
double last_cmd_z = 0.0;
bool last_cmd_valid = false;

double steady_seconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void blackbox_header()
{
    blackbox << "t,kind,reason,incr_x,incr_y,incr_z,off_x,off_y,off_z,"
                "cmd_x,cmd_y,cmd_z,act_x,act_y,act_z,fb_valid,j1,j2,j3,j4,j5,j6,"
                "errcode,ret,held,fb_us\n";
}

void blackbox_line(const char *kind, const char *reason, const CartesianPose &incr,
                   double ox, double oy, double oz,
                   double cx, double cy, double cz, int ret, bool held)
{
    if (!blackbox.is_open())
        return;
    const double t = steady_seconds();
    blackbox << t << ',' << kind << ',' << reason
             << ',' << incr.tran.x << ',' << incr.tran.y << ',' << incr.tran.z
             << ',' << ox << ',' << oy << ',' << oz
             << ',' << cx << ',' << cy << ',' << cz
             << ',' << actual_tcp.tran.x << ',' << actual_tcp.tran.y
             << ',' << actual_tcp.tran.z << ',' << (feedback_valid ? 1 : 0);
    for (int i = 0; i < 6; ++i)
        blackbox << ',' << actual_joints.jVal[i];
    blackbox << ',' << last_errcode << ',' << ret << ',' << (held ? 1 : 0)
             << ',' << last_feedback_us << '\n';
    if (t - blackbox_flush_time > 1.0)
    {
        blackbox.flush();
        blackbox_flush_time = t;
    }
}

template <class F>
double measure_us(F call, int repeats)
{
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < repeats; ++i)
        call();
    const auto dt = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - t0).count();
    return dt / static_cast<double>(repeats);
}

// 部分控制器不实现 get_actual_*，另一些把状态都塞在 get_robot_status 里。
// 启动时各测一次，选可用的、延迟可接受的反馈源。
void calibrate_blackbox_feedback()
{
    feedback_polling = false;
    status_polling = false;
    feedback_source = 0;
    if (dry_run || !blackbox.is_open() || !arm_ok)
        return;

    CartesianPose tcp{};
    JointValue joints{};
    RobotStatus status{};
    RobotStatus_simple simple{};
    const bool tcp_ok = (robot.get_actual_tcp_position(&tcp) == 0);
    const bool joint_ok = (robot.get_actual_joint_position(&joints) == 0);
    const bool status_ok = (robot.get_robot_status(&status) == 0);
    const double tcp_us = measure_us([&]() {robot.get_actual_tcp_position(&tcp);}, 20);
    const double joint_us = measure_us([&]() {robot.get_actual_joint_position(&joints);}, 20);
    const double status_us = measure_us([&]() {robot.get_robot_status(&status);}, 10);
    const double simple_us = measure_us([&]() {robot.get_robot_status_simple(&simple);}, 10);
    const double pair_us = tcp_us + joint_us;

    double motion_us = 0.0;
    if (status_ok && (!tcp_ok || !joint_ok || status_us <= pair_us))
    {
        feedback_source = 2;
        motion_us = status_us;
    }
    else if (tcp_ok && joint_ok)
    {
        feedback_source = 1;
        motion_us = pair_us;
    }

    feedback_stride = std::max(1, std::min(8,
        static_cast<int>(std::ceil(motion_us / 200.0))));
    feedback_polling = feedback_source != 0 && motion_us <= 4000.0;
    status_stride = std::max(13, std::min(200,
        static_cast<int>(std::ceil(simple_us / 100.0))));
    status_polling = simple_us <= 10000.0;

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] black box feedback: status=%.0fus(ok=%d) actual=%.0fus(ok=%d) "
                "joints=%.0fus(ok=%d) simple=%.0fus(ok=%d) source=%s every %d call(s)",
                arm_name.c_str(), status_us, status_ok ? 1 : 0, tcp_us,
                tcp_ok ? 1 : 0, joint_us, joint_ok ? 1 : 0, simple_us,
                status_polling ? 1 : 0,
                feedback_source == 2 ? "robot_status" :
                    (feedback_source == 1 ? "actual_*" : "none"),
                feedback_stride);
    if (!feedback_polling)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] black box cannot sample TCP/joints without delaying servo; "
                    "commands and alarms will still be recorded",
                    arm_name.c_str());
    }
}

void poll_blackbox_feedback()
{
    if (dry_run || !blackbox.is_open() || !arm_ok || rearming)
        return;

    if (feedback_polling &&
        (++feedback_tick % static_cast<unsigned long>(feedback_stride) == 0))
    {
        const double t0 = steady_seconds();
        if (feedback_source == 2)
        {
            RobotStatus status{};
            if (robot.get_robot_status(&status) == 0)
            {
                actual_tcp.tran.x = status.cartesiantran_position[0];
                actual_tcp.tran.y = status.cartesiantran_position[1];
                actual_tcp.tran.z = status.cartesiantran_position[2];
                actual_tcp.rpy.rx = status.cartesiantran_position[3];
                actual_tcp.rpy.ry = status.cartesiantran_position[4];
                actual_tcp.rpy.rz = status.cartesiantran_position[5];
                for (int i = 0; i < 6; ++i)
                    actual_joints.jVal[i] = status.joint_position[i];
                feedback_valid = true;
            }
            else
            {
                feedback_valid = false;
            }
        }
        else if (feedback_source == 1)
        {
            CartesianPose tcp{};
            JointValue joints{};
            const int ret_tcp = robot.get_actual_tcp_position(&tcp);
            const int ret_joints = robot.get_actual_joint_position(&joints);
            if (ret_tcp == 0)
                actual_tcp = tcp;
            if (ret_joints == 0)
                actual_joints = joints;
            feedback_valid = ret_tcp == 0 && ret_joints == 0;
        }
        last_feedback_us = (steady_seconds() - t0) * 1e6;
    }

    if (status_polling &&
        (++status_tick % static_cast<unsigned long>(status_stride) == 0))
    {
        RobotStatus_simple simple{};
        if (robot.get_robot_status_simple(&simple) == 0 &&
            simple.errcode != last_errcode)
        {
            last_errcode = simple.errcode;
            CartesianPose none{};
            blackbox_line("event", last_errcode != 0 ? "alarm" : "alarm_cleared",
                          none, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, false);
            blackbox.flush();
        }
    }
}

// 把 PoseStamped（米 + 四元数）转成控制器位姿（毫米 + 四元数）。
// 这里不做 RPY 转换：每周期增量在 servo_cartesian 里由四元数算出。
bool pose_to_ctrl(const geometry_msgs::msg::PoseStamped &msg,
                  CtrlPose &pose)
{
    // ROS 用米，JAKA SDK 用毫米。
    pose.tran.x = msg.pose.position.x * 1000.0;
    pose.tran.y = msg.pose.position.y * 1000.0;
    pose.tran.z = msg.pose.position.z * 1000.0;

    pose.quat.s = msg.pose.orientation.w;
    pose.quat.x = msg.pose.orientation.x;
    pose.quat.y = msg.pose.orientation.y;
    pose.quat.z = msg.pose.orientation.z;
    Quaternion normalized;
    if (!jaka_teleop::quat_normalize(pose.quat, &normalized)) {
        return false;
    }
    pose.quat = normalized;
    return true;
}

// 按手柄自 Grip 按下以来的位移，向机械臂发一个笛卡尔增量。
// servo_p 每 8 ms 一个 INCR 增量：按 Grip 时的位姿是原点，增量累计起来正好等于
// “原点 + 偏移”；控制器回到原点时臂也回到起点；松开后持续零增量保持位置。
// JAKA 负责笛卡尔到关节的逆解。
//
// 姿态同样相对 Grip 按下时的手柄姿态。节点维护“已经命令给机器人的累计旋转”，
// 因此每周期限速只会拖慢跟随，不会悄悄丢掉一部分快速转动。
void servo_cartesian(JAKAZuRobot &robot,
                     const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                     const CtrlPose &ctrl_origin,
                     CtrlPose &ctrl_prev,
                     const char *arm_name)
{
    CtrlPose ctrl_now;
    if (!pose_to_ctrl(*msg, ctrl_now))
    {
        if (debug)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] ignoring target with invalid orientation quaternion", arm_name);
        }
        return;
    }

    // 工作空间 y 向单侧限位：两臂不得越过共享中心线。
    // 命令的绝对位置是“Grip 按下时 TCP + 手柄偏移”，在算增量之前夹住，
    // 臂正好停在边界，不会先越界再被拉回。
    if (enable_y_limit && arm_tcp_valid)
    {
        ctrl_now.tran.y = gate->apply_workspace_y_limit(
            ctrl_now.tran.y, ctrl_origin.tran.y, arm_is_right,
            arm_tcp_valid, arm_tcp_y_at_grip);
    }

    CartesianPose incr;
    // 每周期平移增量（向量相减是精确的）。
    incr.tran.x = ctrl_now.tran.x - ctrl_prev.tran.x;
    incr.tran.y = ctrl_now.tran.y - ctrl_prev.tran.y;
    incr.tran.z = ctrl_now.tran.z - ctrl_prev.tran.z;

    // 姿态控制：手柄相对 Grip 按下时的旋转。
    Quaternion rotation_step = jaka_teleop::quat_identity();
    Quaternion rotation_step_actual = jaka_teleop::quat_identity();
    bool has_rotation_step = false;
    incr.rpy.rx = 0.0;
    incr.rpy.ry = 0.0;
    incr.rpy.rz = 0.0;
    if (use_rpy_ctrl)
    {
        // Grip 按下以来期望的世界系旋转。
        Quaternion desired_rotation = jaka_teleop::quat_multiply(
            ctrl_now.quat, jaka_teleop::quat_conjugate(ctrl_origin.quat));
        desired_rotation = jaka_teleop::quat_normalized(desired_rotation);
        if (rotation_scale < 1.0)
        {
            const double desired_angle =
                jaka_teleop::quat_rotation_angle(desired_rotation);
            desired_rotation = jaka_teleop::quat_scaled_to_angle(
                desired_rotation, desired_angle * rotation_scale);
        }

        // 还没送到机器人的剩余旋转。
        Quaternion remaining_rotation = jaka_teleop::quat_multiply(
            desired_rotation,
            jaka_teleop::quat_conjugate(commanded_rotation_delta));
        remaining_rotation = jaka_teleop::quat_normalized(remaining_rotation);
        remaining_rotation = jaka_teleop::quat_canonical(remaining_rotation);
        const double remaining_angle =
            jaka_teleop::quat_rotation_angle(remaining_rotation);

        if (remaining_angle >= gate->config().rotation_deadband_rad && rotation_gain > 0.0)
        {
            const double requested_step = remaining_angle * rotation_gain;
            // 先按 0.0015 rad/周期限速，再由许可门按 SLOW 缩放。
            const double applied_step = gate->limit_rotation_step(requested_step);
            rotation_step = jaka_teleop::quat_scaled_to_angle(
                remaining_rotation, applied_step);
            has_rotation_step = applied_step > 1e-6;
        }
    }

    if (has_rotation_step)
    {
        if (dry_run)
        {
            // dry-run 不调用 SDK 的矩阵/四元数转换，只按实际会命令的角度累计。
            rotation_step_actual = rotation_step;
        }
        else
        {
            RotMatrix rot_incr;
            if (robot.quaternion_to_rot_matrix(&rotation_step, &rot_incr) != 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] failed to convert quaternion to rot matrix", arm_name);
                incr.rpy.rx = 0.0;
                incr.rpy.ry = 0.0;
                incr.rpy.rz = 0.0;
                has_rotation_step = false;
            }
            else if (robot.rot_matrix_to_rpy(&rot_incr, &incr.rpy) != 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] failed to convert rot matrix to RPY", arm_name);
                incr.rpy.rx = 0.0;
                incr.rpy.ry = 0.0;
                incr.rpy.rz = 0.0;
                has_rotation_step = false;
            }
            else
            {
                incr.rpy.rx *= rotation_sign;
                incr.rpy.ry *= rotation_sign;
                incr.rpy.rz *= rotation_sign;
                // 用实际发给控制器的带符号 RPY 重建旋转；这样 rotation_sign
                // 改变或欧拉转换换轴时，累计参考仍然正确。
                RotMatrix actual_rot;
                Quaternion actual_quat;
                if (robot.rpy_to_rot_matrix(&incr.rpy, &actual_rot) != 0 ||
                    robot.rot_matrix_to_quaternion(&actual_rot, &actual_quat) != 0 ||
                    !jaka_teleop::quat_normalize(actual_quat, &rotation_step_actual))
                {
                    RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                                "[%s] failed to verify orientation increment", arm_name);
                    incr.rpy.rx = 0.0;
                    incr.rpy.ry = 0.0;
                    incr.rpy.rz = 0.0;
                    has_rotation_step = false;
                }
            }
        }
    }

    // 死区：亚毫米/亚度的抖动不追，原地保持。
    double max_tran = std::max(std::fabs(incr.tran.x),
                      std::max(std::fabs(incr.tran.y), std::fabs(incr.tran.z)));
    if (max_tran < gate->config().translation_deadband_mm && !has_rotation_step)
        return;

    // 5 mm/周期限速 + SLOW 缩放。注意 ctrl_prev 仍然更新到手柄最新位置，
    // 所以 SLOW/REVOKED 期间丢掉的跟随量不会在恢复后一次性补上（无跳变）。
    vr_safe_arm::Vec3 raw_incr;
    raw_incr.x = incr.tran.x;
    raw_incr.y = incr.tran.y;
    raw_incr.z = incr.tran.z;
    const vr_safe_arm::Vec3 limited_incr = gate->limit_translation_step(raw_incr);
    incr.tran.x = limited_incr.x;
    incr.tran.y = limited_incr.y;
    incr.tran.z = limited_incr.z;

    poll_blackbox_feedback();
    const double off_x = ctrl_now.tran.x - ctrl_origin.tran.x;
    const double off_y = ctrl_now.tran.y - ctrl_origin.tran.y;
    const double off_z = ctrl_now.tran.z - ctrl_origin.tran.z;
    double cmd_x = 0.0;
    double cmd_y = 0.0;
    double cmd_z = 0.0;
    if (arm_tcp_valid)
    {
        cmd_x = arm_tcp_at_grip_pose.tran.x + off_x;
        cmd_y = arm_tcp_at_grip_pose.tran.y + off_y;
        cmd_z = arm_tcp_at_grip_pose.tran.z + off_z;
        last_cmd_valid = true;
    }
    else if (feedback_valid)
    {
        cmd_x = actual_tcp.tran.x;
        cmd_y = actual_tcp.tran.y;
        cmd_z = actual_tcp.tran.z;
        last_cmd_valid = true;
    }
    if (last_cmd_valid)
    {
        last_cmd_x = cmd_x;
        last_cmd_y = cmd_y;
        last_cmd_z = cmd_z;
    }

    if (dry_run)
    {
        // dry-run：只累积模拟 TCP，不调用 servo_p。
        sim_tcp_x += incr.tran.x;
        sim_tcp_y += incr.tran.y;
        sim_tcp_z += incr.tran.z;
        last_cmd_x = sim_tcp_x;
        last_cmd_y = sim_tcp_y;
        last_cmd_z = sim_tcp_z;
        blackbox_line("servo", "dry_run_sent", incr, off_x, off_y, off_z,
                      sim_tcp_x, sim_tcp_y, sim_tcp_z, 0, false);
    }
    else
    {
        int ret = robot.servo_p(&incr, MoveMode::INCR);
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] servo_p failed: %s", arm_name, mapErr[ret].c_str());
            blackbox_line("servo", "failed", incr, off_x, off_y, off_z,
                          cmd_x, cmd_y, cmd_z, ret, false);
            return;
        }
        blackbox_line("servo", "sent", incr, off_x, off_y, off_z,
                      cmd_x, cmd_y, cmd_z, ret, false);
    }

    // 记录实际命令到了哪里，下一周期的增量从机器人用的同一参考算起。
    ctrl_prev = ctrl_now;
    if (has_rotation_step)
    {
        commanded_rotation_delta = jaka_teleop::quat_multiply(
            rotation_step_actual, commanded_rotation_delta);
        commanded_rotation_delta =
            jaka_teleop::quat_normalized(commanded_rotation_delta);
        commanded_rotation_delta =
            jaka_teleop::quat_canonical(commanded_rotation_delta);
    }

    // 调试：限频打印总平移偏移和每周期增量旋转。
    static int dbg_count = 0;
    if (++dbg_count % 25 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] offset=(%.1f,%.1f,%.1f)mm incr=(%.2f,%.2f,%.2f)mm rincr=(%.4f,%.4f,%.4f)rad",
                    arm_name,
                    ctrl_now.tran.x - ctrl_origin.tran.x,
                    ctrl_now.tran.y - ctrl_origin.tran.y,
                    ctrl_now.tran.z - ctrl_origin.tran.z,
                    incr.tran.x, incr.tran.y, incr.tran.z,
                    incr.rpy.rx, incr.rpy.ry, incr.rpy.rz);
    }
}

// 发一个零增量保持位置。JAKA servo_p 必须持续喂；一次零增量不是可靠刹车，
// 所以要由 8 ms 定时器持续发（hold stream）。
bool servo_hold(JAKAZuRobot &robot)
{
    CartesianPose hold;
    hold.tran.x = 0.0;
    hold.tran.y = 0.0;
    hold.tran.z = 0.0;
    hold.rpy.rx = 0.0;
    hold.rpy.ry = 0.0;
    hold.rpy.rz = 0.0;
    poll_blackbox_feedback();
    if (dry_run)
    {
        blackbox_line("servo", "hold_dry_run", hold, 0.0, 0.0, 0.0,
                      last_cmd_x, last_cmd_y, last_cmd_z, 0, true);
        return true;
    }
    int ret = robot.servo_p(&hold, MoveMode::INCR);
    if (ret != 0)
    {
        blackbox_line("servo", "hold_failed", hold, 0.0, 0.0, 0.0,
                      last_cmd_x, last_cmd_y, last_cmd_z, ret, true);
        if (debug)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] servo_p HOLD failed: %s", arm_name.c_str(), mapErr[ret].c_str());
        }
        return false;
    }
    blackbox_line("servo", "hold", hold, 0.0, 0.0, 0.0,
                  last_cmd_x, last_cmd_y, last_cmd_z, ret, true);
    return true;
}

// 读当前 TCP。dry-run 用模拟值；真机在读不到或被重锚定占用时返回 false。
bool read_tcp_sample(double *x, double *y, double *z)
{
    if (dry_run)
    {
        *x = sim_tcp_x;
        *y = sim_tcp_y;
        *z = sim_tcp_z;
        return true;
    }
    if (!arm_ok || rearming)
        return false;
    CartesianPose tcp{};
    if (robot.get_tcp_position(&tcp) != 0)
        return false;
    *x = tcp.tran.x;
    *y = tcp.tran.y;
    *z = tcp.tran.z;
    return true;
}

// 10 Hz 采样一次 TCP，差分得到末端线速度（m/s）。采样失败超过 0.5 s 则标记无效，
// 避免拿旧速度冒充“已停稳”。
void update_tcp_speed()
{
    const double now = steady_seconds();
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!read_tcp_sample(&x, &y, &z))
    {
        if (tcp_speed_valid && now - tcp_speed_stamp > 0.5)
            tcp_speed_valid = false;
        return;
    }

    if (has_tcp_sample)
    {
        const double dt = now - last_tcp_sample_t;
        if (dt > 1e-4)
        {
            const double dx = x - last_tcp_sample_x;
            const double dy = y - last_tcp_sample_y;
            const double dz = z - last_tcp_sample_z;
            const double distance_mm = std::sqrt(dx * dx + dy * dy + dz * dz);
            tcp_speed_mps = (distance_mm / 1000.0) / dt;
            tcp_speed_stamp = now;
            tcp_speed_valid = true;
        }
    }
    else
    {
        has_tcp_sample = true;
    }
    last_tcp_sample_t = now;
    last_tcp_sample_x = x;
    last_tcp_sample_y = y;
    last_tcp_sample_z = z;
}

void publish_arm_state()
{
    if (!g_node || !arm_state_pub || !gate)
        return;
    const double reported_speed = tcp_speed_valid ? tcp_speed_mps : 0.0;
    vr_safe_interfaces::msg::ArmState msg;
    msg.header.stamp = g_node->now();
    msg.header.frame_id = "base_link";
    msg.arm = arm_name;
    // REVOKED 或驱动未就绪都算“已阻断新指令”。
    msg.command_blocked = gate->command_blocked() || !arm_ok;
    msg.motion_stopped = gate->motion_stopped(
        reported_speed, tcp_speed_valid,
        hold_stream_enabled.load() && !gate->gripped());
    msg.tcp_speed_mps = static_cast<float>(reported_speed);
    msg.state = gate->state_name(dry_run, arm_ok);
    arm_state_pub->publish(msg);
}

// JAKA servo_p 必须持续喂。Grip 松开、许可 REVOKED、看门狗触发后都保持零增量流。
void hold_stream_callback()
{
    // 重锚定线程可能正在 login/power_on/servo_move_enable，JAKA SDK 不支持并发调用，
    // 因此这段时间暂停 hold stream；重锚定结束后由 rearm_worker 重新打开。
    if (!arm_ok || rearming)
        return;
    const double now = steady_seconds();
    const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const int64_t last_permit = last_permit_ms.load();
    if (last_permit > 0 && (now_ms - last_permit) > permit_watchdog_ms &&
        gate->permit() != vr_safe_arm::PermitDecision::kRevoked)
    {
        gate->update_permit(vr_safe_arm::PermitDecision::kRevoked,
                            "permit timeout", gate->permit_seq());
        hold_stream_enabled = true;
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] %.0f ms 没有新许可，按失联处理：阻断 target 并保持位置",
                    arm_name.c_str(), permit_watchdog_ms);
    }

    // 本地 target 看门狗：gripped 时 200 ms 没有新 target，按跟踪丢失处理。
    const bool watchdog_tripped = gate->check_target_watchdog(now);
    if (watchdog_tripped)
    {
        hold_stream_enabled = true;
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] %.0f ms 没有新的 target，判定跟踪丢失：松开离合并保持位置",
                    arm_name.c_str(), gate->config().target_watchdog_s * 1000.0);
    }

    // 许可阻断时无条件保持零增量流（不依赖上一条状态）。
    if (gate->command_blocked())
        hold_stream_enabled = true;

    poll_blackbox_feedback();
    if (gate->gripped() || !hold_stream_enabled)
    {
        if (watchdog_tripped)
            publish_arm_state();
        return;
    }

    if (!servo_hold(robot))
    {
        ++hold_stream_failure_count;
        if (hold_stream_failure_count >= 5)
        {
            hold_stream_enabled = false;
            hold_stream_failure_count = 0;
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] stopping hold stream after repeated servo_p failures",
                        arm_name.c_str());
        }
    }
    else
    {
        hold_stream_failure_count = 0;
    }

    // 看门狗触发后立即发布一次状态，不等 10 Hz 定时器。
    if (watchdog_tripped)
        publish_arm_state();
}

// 从保护性停机/碰撞/掉伺服里把臂恢复过来。按“伺服 -> 使能 -> 上电”自上而下
// 检查，自下而上执行，只做真正缺的步骤：
//   - 已在伺服 -> 什么都不做；
//   - 碰撞/报警 -> collision_recover() + clear_error()；
//   - 未上电     -> power_on（等 8 s）；
//   - 未使能     -> enable_robot（等 4 s）；
//   - 其他情况   -> 重新进伺服。
bool ensure_arm_ready(JAKAZuRobot &robot)
{
    if (!arm_ok)
        return false;
    if (dry_run)
        return true;

    // 快路径：仍在伺服，无需恢复。
    BOOL in_servo = FALSE;
    if (robot.is_in_servomove(&in_servo) == 0 && in_servo)
        return true;

    // 先清保护停/碰撞/运动异常，否则控制器不接受 power_on/enable/servo。
    BOOL in_collision = FALSE;
    if (robot.is_in_collision(&in_collision) == 0 && in_collision)
    {
        int r = robot.collision_recover();
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] collision_recover ret=%d", arm_name.c_str(), r);
    }
    int r = robot.clear_error();
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] clear_error ret=%d", arm_name.c_str(), r);

    // 读上电/使能标志；get_robot_status_simple 是两者最轻量的来源。
    RobotStatus_simple status{};
    if (robot.get_robot_status_simple(&status) != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] get_robot_status_simple failed; cannot re-arm", arm_name.c_str());
        return false;
    }

    if (status.powered_on == 0)
    {
        int ret = robot.power_on();
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] re-arm power_on FAILED (ret=%d: %s)", arm_name.c_str(),
                        ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm power_on OK", arm_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(8));
        // 上电后机器人仍是未使能，刷新标志再决定是否需要 enable_robot。
        if (robot.get_robot_status_simple(&status) != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] get_robot_status_simple failed after power_on", arm_name.c_str());
            return false;
        }
    }

    if (status.enabled == 0)
    {
        int ret = robot.enable_robot();
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] re-arm enable_robot FAILED (ret=%d: %s)", arm_name.c_str(),
                        ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm enable_robot OK", arm_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }

    int ret = robot.servo_move_enable(true);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] re-arm servo_move_enable FAILED (ret=%d: %s)", arm_name.c_str(),
                    ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm servo enabled", arm_name.c_str());
    return true;
}

// 后台重锚定。ensure_arm_ready 的慢序列（上电/使能各等数秒）不能跑在执行器线程，
// 否则 target 回调会被冻结 12 s 以上；也不能和 servo_p 并发，所以重锚定期间
// rearming=true，目标回调和 hold stream 都绕开 SDK。
void rearm_worker()
{
    bool ok = ensure_arm_ready(robot);

    // 只有仍按着 Grip、许可允许、且没有被要求“重新按”时，才捕获新原点。
    bool should_track = ok && gate->grip_pressed() &&
        gate->tracking_allowed() && !gate->regrip_required();

    if (should_track)
    {
        CtrlPose origin;
        {
            std::lock_guard<std::mutex> lk(latest_mtx);
            if (!pose_to_ctrl(latest_pose, origin))
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] cannot start tracking without a valid controller orientation",
                            arm_name.c_str());
                ok = false;
            }
        }
        if (ok)
        {
            // 按 Grip 时记录一次绝对 TCP y，工作空间限位用它算“Grip TCP + 手柄偏移”，
            // 不依赖每周期读回，因此没有滞后和边界回弹。
            CartesianPose tcp_at_grip{};
            last_cmd_valid = false;
            if (dry_run)
            {
                arm_tcp_valid = true;
                tcp_at_grip.tran.x = sim_tcp_x;
                tcp_at_grip.tran.y = sim_tcp_y;
                tcp_at_grip.tran.z = sim_tcp_z;
            }
            else
            {
                arm_tcp_valid = (robot.get_tcp_position(&tcp_at_grip) == 0);
            }
            if (arm_tcp_valid)
            {
                arm_tcp_y_at_grip = tcp_at_grip.tran.y;
                arm_tcp_at_grip_pose = tcp_at_grip;
                last_cmd_x = tcp_at_grip.tran.x;
                last_cmd_y = tcp_at_grip.tran.y;
                last_cmd_z = tcp_at_grip.tran.z;
                last_cmd_valid = true;
            }
            // 先写好 origin/prev，再让 gate 置 gripped=true，目标回调才会开始用它们。
            ctrl_origin = origin;
            ctrl_prev = origin;
            commanded_rotation_delta = jaka_teleop::quat_identity();
        }
    }

    // 所有 SDK 调用都已结束，先放开 rearming 给 hold stream，再请求进入跟踪。
    rearming = false;
    const bool engaged = gate->finish_rearm(ok && should_track, steady_seconds());
    if (engaged)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] gripped, ctrl_origin=(%.1f,%.1f,%.1f)mm, 6D=%s",
                    arm_name.c_str(),
                    ctrl_origin.tran.x, ctrl_origin.tran.y, ctrl_origin.tran.z,
                    use_rpy_ctrl ? "true" : "false");
    }
    else
    {
        // 重锚定失败/许可被撤/已松开：继续零增量流，等下一次按下。
        hold_stream_enabled = true;
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] re-arm did not engage tracking; holding position", arm_name.c_str());
    }
}

// 处理本臂 Grip。按下/松开阈值构成 Schmitt 触发，防止 0.5 附近抖动导致反复离合。
// 许可恢复后不会自动跟随：gate 的 regrip_required 门闩要求先松开再按。
void handle_grip(float value)
{
    const auto action = gate->update_grip(value, steady_seconds());
    switch (action)
    {
        case vr_safe_arm::GripAction::kStartReArm:
        {
            hold_stream_enabled = false;
            // 回收上一次已经结束的重锚定线程，再启动新的（给 joinable thread 赋值
            // 会直接 terminate）。
            if (rearm_thread.joinable())
                rearm_thread.join();
            rearming = true;
            rearm_thread = std::thread(rearm_worker);
            break;
        }
        case vr_safe_arm::GripAction::kReleaseAndHold:
        {
            hold_stream_enabled = true;
            if (arm_ok && !rearming)
                servo_hold(robot);
            publish_arm_state();
            break;
        }
        case vr_safe_arm::GripAction::kWaitForRelease:
            // 每帧 Joy 都会走到这里；保持 HOLD，不刷日志。
            hold_stream_enabled = true;
            break;
        case vr_safe_arm::GripAction::kNone:
        default:
            break;
    }
}

void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!arm_ok)
        return;
    {
        std::lock_guard<std::mutex> lk(latest_mtx);
        latest_pose = *msg;
    }
    gate->note_target(steady_seconds());

    // 许可 REVOKED：阻断新目标，不发 servo_p 增量；hold stream 保持零增量。
    if (gate->command_blocked())
    {
        hold_stream_enabled = true;
        return;
    }

    if (gate->gripped())
    {
        servo_cartesian(robot, msg, ctrl_origin, ctrl_prev, arm_name.c_str());
    }
    // 未跟踪时由独立的 125 Hz hold stream 持续发零增量。

    if(debug)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] target_callback: %f,%f,%f",
                    arm_name.c_str(), msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    }
}

void button_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float grip = msg->axes.size() > static_cast<size_t>(grip_axis)
                     ? msg->axes[grip_axis] : 0.0f;
    if (arm_ok)
        handle_grip(grip);

    if (gripper != nullptr)
    {
        // 许可 REVOKED 时夹爪也冻结：保持最后位置/力，忽略锁定键和扳机。
        // 但仍记录当前扳机值，避免许可恢复后第一帧拿旧值去追一个跳变。
        if (gate->command_blocked())
        {
            if (gripper_linear && msg->axes.size() > static_cast<size_t>(gripper_axis))
            {
                float t = msg->axes[gripper_axis];
                prev_gripper_trigger = std::max(0.0f, std::min(1.0f, t));
            }
            return;
        }

        // 按本臂的锁定键翻转夹爪冻结：左手柄 X（Joy.buttons[0]），右手柄 A
        // （Joy.buttons[2]）。上升沿翻转，冻结期间忽略扳机轴。
        bool lock_button =
            msg->buttons.size() > static_cast<size_t>(gripper_lock_button_index) &&
            msg->buttons[gripper_lock_button_index] != 0;
        if (lock_button && !prev_lock_button)
        {
            gripper_locked = !gripper_locked;
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI gripper %s",
                        arm_name.c_str(),
                        gripper_locked ? "locked" : "unlocked");
        }
        prev_lock_button = lock_button;

        // 两种夹爪控制模式：
        // - digital：按键二值控制 -> set_closed(bool)
        // - linear ：扳机模拟量 -> Grip(float)
        if (!gripper_linear)
        {
            bool pressed = msg->buttons.size() > static_cast<size_t>(gripper_button_index) &&
                           msg->buttons[gripper_button_index] != 0;
            if (pressed != gripper->closed())
            {
                std::string error;
                if (!gripper->set_closed(pressed, &error))
                {
                    RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper command failed: %s",
                                arm_name.c_str(), error.c_str());
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper %s",
                                arm_name.c_str(), pressed ? "closed" : "opened");
                }
            }
        }
        else // 模拟量
        {
            // 冻结：保持最后指令位置和力，忽略扳机轴，直到再按一次锁定键。
            if (gripper_locked)
                return;

            float trigger_raw = msg->axes.size() > static_cast<size_t>(gripper_axis)
                                  ? msg->axes[gripper_axis] : 0.0f;
            float trigger = trigger_raw;
            if (trigger < 0.0f) trigger = 0.0f;
            if (trigger > 1.0f) trigger = 1.0f;
            if (std::fabs(trigger - prev_gripper_trigger) >= gripper_trigger_deadband)
            {
                std::string error;
                if (!gripper->Grip(trigger, &error))
                {
                    RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper Grip(%.3f) failed: %s",
                                arm_name.c_str(), trigger, error.c_str());
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper Grip(%.3f)",
                                arm_name.c_str(), trigger);
                }
                prev_gripper_trigger = trigger;
            }
        }
    }
}

// 把一台臂完整上线：login -> 配置 TIO -> power_on -> enable -> servo。
// 与 jaka_driver 参考序列一致（power_on 后等 8 s、enable 后等 4 s）。
// 任一步失败返回 false，回调会跳过该臂。dry-run 直接返回成功且不碰 SDK。
bool bring_up_arm(JAKAZuRobot &robot, const std::string &ip, const char *arm)
{
    if (dry_run)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "%s dry_run: 跳过 login/power_on/enable_robot/servo_move_enable/PGI 初始化",
                    arm);
        return true;
    }

    int ret = robot.login_in(ip.c_str());
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s login FAILED (%s, ret=%d: %s)",
                    arm, ip.c_str(), ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s login OK (%s)", arm, ip.c_str());

    // login 后立即做与参考节点相同的 SDK 设置。
    robot.set_status_data_update_time_interval(100);
    robot.set_block_wait_timeout(120);

    // 在进入伺服模式前配置 TIO；JAKA 控制器进入伺服后可能拒绝改引脚模式。
    if (gripper != nullptr)
    {
        std::string error;
        if (!gripper->initialize(&error))
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI gripper disabled: %s", arm_name.c_str(), error.c_str());
            gripper.reset();
        }
        else
        {
            const auto & config = gripper->config();
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI-140-80 ready: RS485 channel=%d slave=%d force=%d%% speed=%d%% "
                        "open=%d closed=%d trigger_axis=%d mode=%s button=%d deadband=%.3f",
                        arm_name.c_str(), config.rs485_channel, config.slave_id,
                        config.force_percent, config.speed_percent,
                        config.open_position, config.closed_position,
                        gripper_axis, (gripper_linear ? "linear" : "digital"),
                        (gripper_linear ? -1 : gripper_button_index),
                        gripper_trigger_deadband);
        }
    }

    ret = robot.power_on();
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s power_on FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s power_on OK", arm);
    std::this_thread::sleep_for(std::chrono::seconds(8));

    ret = robot.enable_robot();
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s enable_robot FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s enable_robot OK", arm);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // 保留碰撞检测，但把阈值 25N -> 75N（level 3）。关掉碰撞（level 0）会让
    // 机械臂直接顶穿障碍物，VR 遥操作下不安全。
    robot.set_collision_level(3);

    // 给 servo_p 加硬性笛卡尔速度/加速度/加加速度上限（mm/s、mm/s^2、mm/s^3）
    // 与姿态上限（deg/s 等）：在软件每周期限速之外再加一层安全边界。
    // 非线性滤波，参数沿用基线。
    ret = robot.servo_move_use_carte_NLF(1500.0, 2500.0, 2800.0,1120.0, 1180.0, 1720.0);
    if(ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s servo_move_use_carte_NLF FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }

    ret = robot.servo_move_enable(true);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s servo_move_enable FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    BOOL in_servo = FALSE;
    robot.is_in_servomove(&in_servo);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s servo enabled (is_in_servomove=%d)", arm, in_servo);
    return true;
}

// 许可回调：把 MotionPermit 翻成 gate 的三值决定。
// REVOKED 时立刻松离合、开 hold stream，并保持伺服使能（不 disable servo）。
void permit_callback(const vr_safe_interfaces::msg::MotionPermit::SharedPtr msg)
{
    vr_safe_arm::PermitDecision decision = vr_safe_arm::PermitDecision::kRevoked;
    if (msg->decision == vr_safe_interfaces::msg::MotionPermit::GRANTED)
        decision = vr_safe_arm::PermitDecision::kGranted;
    else if (msg->decision == vr_safe_interfaces::msg::MotionPermit::SLOW)
        decision = vr_safe_arm::PermitDecision::kSlow;
    else if (msg->decision != vr_safe_interfaces::msg::MotionPermit::REVOKED)
    {
        // 未知值按最坏情况处理，避免协议扩展时静默放行。
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] 未知 MotionPermit.decision=%u，按 REVOKED 处理",
                    arm_name.c_str(), static_cast<unsigned>(msg->decision));
    }

    last_permit_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    const auto before = gate->permit();
    gate->update_permit(decision, msg->reason, static_cast<uint32_t>(msg->seq));

    if (decision == vr_safe_arm::PermitDecision::kRevoked)
    {
        hold_stream_enabled = true;
        if (arm_ok && !rearming)
            servo_hold(robot);
        if (before != vr_safe_arm::PermitDecision::kRevoked)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] 许可 REVOKED（%s）：阻断新 target，松开离合，持续零增量；"
                        "恢复后需重新按 Grip",
                        arm_name.c_str(), msg->reason.c_str());
        }
    }
    else if (before == vr_safe_arm::PermitDecision::kRevoked)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] 许可恢复为 %s（%s）：不会自动跟随，请松开并重新按 Grip",
                    arm_name.c_str(), vr_safe_arm::permit_decision_name(decision),
                    msg->reason.c_str());
    }
    else if (decision == vr_safe_arm::PermitDecision::kSlow)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] 许可 SLOW：平移/姿态步长按 %.2f 缩放",
                    arm_name.c_str(), gate->config().slow_scale);
    }

    publish_arm_state();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "C");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rc_ctrl_node");
    g_node = node;

    // 本进程控制哪台臂。JAKA SDK 每进程一条连接，因此一个实例只驱动一台臂，
    // launch 起两个实例（arm_name left/right）。
    arm_name = node->declare_parameter<string>("arm_name", "left");
    bool is_right = (arm_name == "right");
    arm_is_right = is_right;
    // 夹爪锁定键：左手柄 X（buttons[0]），右手柄 A（buttons[2]）。
    gripper_lock_button_index = is_right ? 2 : 0;
    string default_ip = is_right ? "192.168.71.36" : "192.168.71.37";
    string default_target = is_right ?
        "/vr_safe/teleop/right_target" : "/vr_safe/teleop/left_target";
    string default_state = is_right ?
        "/vr_safe/arm/right/state" : "/vr_safe/arm/left/state";
    int default_grip_axis = is_right ? 1 : 0;

    // 连接/话题参数。默认值与 VR_safe 的契约话题一致。
    string ip = node->declare_parameter<string>("robot_ip", default_ip);
    string target_topic = node->declare_parameter<string>("target_topic", default_target);
    string button_topic = node->declare_parameter<string>(
        "button_topic", "/vr_safe/teleop/button");
    string permit_topic = node->declare_parameter<string>(
        "permit_topic", "/vr_safe/permit");
    string state_topic = node->declare_parameter<string>("state_topic", default_state);
    grip_axis = node->declare_parameter<int>("grip_axis", default_grip_axis);
    dry_run = node->declare_parameter<bool>("dry_run", false);

    // PGI-140-80 默认由 Quest 食指扳机轴控制（axes[2]/[3]）；也支持数字按键。
    gripper_axis = node->declare_parameter<int>(
        "gripper_axis", is_right ? 3 : 2);
    gripper_use_button = node->declare_parameter<bool>("gripper_use_button", false);
    const int configured_button_index = node->declare_parameter<int>(
        "gripper_button_index", -1);
    // Joy.buttons 沿用 quest_reader 的稳定顺序：LTr=10、RTr=11。
    gripper_button_index = configured_button_index >= 0 ? configured_button_index :
        (is_right ? 11 : 10);
    gripper_linear = node->declare_parameter<bool>("gripper_linear", true);
    gripper_trigger_deadband = node->declare_parameter<double>("gripper_trigger_deadband", 0.02);
    // 向后兼容：旧布尔参数存在时优先数字模式。
    if (gripper_use_button) {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] parameter gripper_use_button is present; using digital mode",
                    arm_name.c_str());
        gripper_linear = false;
    }

    pgi140::GripperConfig gripper_config;
    gripper_config.rs485_channel = node->declare_parameter<int>("gripper_rs485_channel", 1);
    gripper_config.slave_id = node->declare_parameter<int>("gripper_slave_id", 1);
    gripper_config.baudrate = node->declare_parameter<int>("gripper_baudrate", 115200);
    gripper_config.force_percent = node->declare_parameter<int>("gripper_force", 20);
    gripper_config.speed_percent = node->declare_parameter<int>("gripper_speed", 100);
    gripper_config.open_position = node->declare_parameter<int>("gripper_open_position", 1000);
    gripper_config.closed_position = node->declare_parameter<int>(
        "gripper_closed_position", 0);
    gripper_config.initialize = node->declare_parameter<bool>("gripper_initialize", true);
    gripper_config.initialize_command = node->declare_parameter<int>(
        "gripper_initialize_command", 0x01);
    gripper_config.initialize_delay_ms = node->declare_parameter<int>(
        "gripper_initialize_delay_ms", 2000);
    gripper_config.configure_tio = node->declare_parameter<bool>(
        "gripper_configure_tio", false);
    gripper_config.enable_tio_power = node->declare_parameter<bool>(
        "gripper_enable_tio_power", true);
    gripper_config.tio_voltage = node->declare_parameter<int>(
        "gripper_tio_voltage", 0);

    debug = node->declare_parameter<bool>("debug", false);
    use_rpy_ctrl = node->declare_parameter<bool>("use_rpy_ctrl", true);
    rotation_scale = node->declare_parameter<double>("rotation_scale", 0.5);
    rotation_gain = node->declare_parameter<double>("rotation_gain", 0.5);
    rotation_deadband = node->declare_parameter<double>(
        "rotation_deadband_rad", 0.005);
    rotation_max_step = node->declare_parameter<double>(
        "rotation_max_step_rad", 0.0015);
    rotation_sign = node->declare_parameter<int>("rotation_sign", 1);
    grip_press_threshold = node->declare_parameter<double>(
        "grip_press_threshold", 0.6);
    grip_release_threshold = node->declare_parameter<double>(
        "grip_release_threshold", 0.4);
    enable_y_limit = node->declare_parameter<bool>("enable_y_limit", true);

    // 安全门参数：slow_scale 与本地看门狗都来自工单 05 / design.md。
    const double slow_scale = node->declare_parameter<double>("slow_scale", 0.3);
    const double target_watchdog_ms = node->declare_parameter<double>(
        "target_watchdog_ms", 200.0);
    permit_watchdog_ms = node->declare_parameter<double>(
        "permit_watchdog_ms", 300.0);
    const double state_rate_hz = node->declare_parameter<double>(
        "state_publish_rate_hz", 10.0);
    const double stopped_speed_mps = node->declare_parameter<double>(
        "stopped_speed_mps", 0.005);

    rotation_scale = std::max(0.0, std::min(1.0, rotation_scale));
    rotation_gain = std::max(0.0, std::min(1.0, rotation_gain));
    rotation_deadband = std::max(0.0, rotation_deadband);
    if (rotation_max_step <= 0.0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] rotation_max_step_rad must be positive; using 0.0015",
                    arm_name.c_str());
        rotation_max_step = 0.0015;
    }
    if (rotation_sign != 1 && rotation_sign != -1)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] rotation_sign must be 1 or -1; using 1",
                    arm_name.c_str());
        rotation_sign = 1;
    }
    grip_press_threshold = std::max(0.0, std::min(1.0, grip_press_threshold));
    grip_release_threshold = std::max(0.0, std::min(1.0, grip_release_threshold));
    if (grip_press_threshold <= grip_release_threshold)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] grip_press_threshold must exceed grip_release_threshold; "
                    "using 0.6/0.4", arm_name.c_str());
        grip_press_threshold = 0.6;
        grip_release_threshold = 0.4;
    }

    // 构造安全闸门。gate 内部还会再收敛一次参数，NaN/负数不会进入控制回路。
    vr_safe_arm::ArmCoreConfig core_config;
    core_config.slow_scale = slow_scale;
    core_config.target_watchdog_s = target_watchdog_ms / 1000.0;
    core_config.grip_press_threshold = grip_press_threshold;
    core_config.grip_release_threshold = grip_release_threshold;
    core_config.max_translation_step_mm = 5.0;
    core_config.translation_deadband_mm = 0.5;
    core_config.rotation_deadband_rad = rotation_deadband;
    core_config.max_rotation_step_rad = rotation_max_step;
    core_config.stopped_speed_mps = stopped_speed_mps;
    core_config.y_limit_mm = 110.0;
    gate = std::make_unique<vr_safe_arm::ArmSafetyGate>(core_config);

    // 黑匣子：每次 servo_p 落盘，便于现场复盘；打不开不影响控制。
    const std::string blackbox_dir = node->declare_parameter<std::string>(
        "servo_blackbox_dir", std::string("log/teleop"));
    if (!blackbox_dir.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(blackbox_dir, ec);
        const auto stamp = std::to_string(std::chrono::duration_cast<
            std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        const std::string path = blackbox_dir + "/" + arm_name + "_" + stamp + ".csv";
        blackbox.open(path, std::ios::out | std::ios::trunc);
        if (blackbox.is_open())
        {
            blackbox << std::setprecision(12);
            blackbox_header();
            blackbox.flush();
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] black box: %s", arm_name.c_str(), path.c_str());
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] cannot open black box '%s'; continuing without it",
                        arm_name.c_str(), path.c_str());
        }
    }

    // dry_run：跳过所有 JAKA/PGI 调用，但保留完整的许可/离合/看门狗状态机。
    if (dry_run)
    {
        gripper.reset();
        arm_ok = true;
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] dry_run=true：不连接 JAKA，不调用 login/power/enable/servo/gripper；"
                    "permit/target/ArmState 照常处理", arm_name.c_str());
    }
    else
    {
        gripper = std::make_unique<pgi140::Pgi140Gripper>(robot, gripper_config);
        arm_ok = bring_up_arm(robot, ip, arm_name.c_str());
        if (!arm_ok)
        {
            gripper.reset();
            RCLCPP_FATAL(rclcpp::get_logger("rc_ctrl"),
                         "[%s] JAKA 启动失败（%s）：节点退出，交由 launch 中止整套系统",
                         arm_name.c_str(), ip.c_str());
            if (blackbox.is_open())
            {
                blackbox.flush();
                blackbox.close();
            }
            rclcpp::shutdown();
            return 1;
        }
        else
        {
            calibrate_blackbox_feedback();
        }
    }

    // 启动后先按 REVOKED 对待：保持伺服使能，但持续发零增量。
    hold_stream_enabled = true;

    // 状态发布（10 Hz，RELIABLE depth 1）。
    arm_state_pub = node->create_publisher<vr_safe_interfaces::msg::ArmState>(
        state_topic, rclcpp::QoS(1));

    // 许可订阅：RELIABLE + TRANSIENT_LOCAL depth 1，晚启动也能拿到最后一条许可。
    auto permit_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto permit_sub = node->create_subscription<vr_safe_interfaces::msg::MotionPermit>(
        permit_topic, permit_qos, permit_callback);

    // 目标/按键订阅：keep_last(1)，控制循环只关心最新一帧。
    auto target_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic, rclcpp::QoS(1), target_callback);
    auto button_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        button_topic, rclcpp::QoS(1), button_callback);

    // 125 Hz hold stream：不断喂零增量，也是本地 target 看门狗的执行点。
    auto hold_timer = node->create_wall_timer(
        std::chrono::milliseconds(8), hold_stream_callback);

    // 10 Hz 状态采样/发布。
    const double rate = (std::isfinite(state_rate_hz) && state_rate_hz > 0.0) ?
        std::max(1.0, std::min(100.0, state_rate_hz)) : 10.0;
    auto state_timer = node->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
        []() {
            update_tcp_speed();
            publish_arm_state();
        });
    (void)permit_sub;
    (void)hold_timer;
    (void)state_timer;

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "rc_ctrl_node started: arm=%s, ip=%s, dry_run=%s, tracking %s "
                "(grip axis %d), permit=%s, state=%s",
                arm_name.c_str(), ip.c_str(), dry_run ? "true" : "false",
                target_topic.c_str(), grip_axis, permit_topic.c_str(),
                state_topic.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] permit gate: slow_scale=%.2f target_watchdog=%.0f ms "
                "y_limit=%.0f mm max_step=%.1f mm/cycle",
                arm_name.c_str(), gate->config().slow_scale,
                gate->config().target_watchdog_s * 1000.0,
                gate->config().y_limit_mm, gate->config().max_translation_step_mm);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] orientation control: enabled=%s scale=%.2f gain=%.2f "
                "deadband=%.4f rad max_step=%.4f rad sign=%d",
                arm_name.c_str(), use_rpy_ctrl ? "true" : "false",
                rotation_scale, rotation_gain, rotation_deadband,
                rotation_max_step, rotation_sign);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] grip hysteresis: press>%.2f release<%.2f; "
                "continuous 125 Hz hold when disengaged",
                arm_name.c_str(), grip_press_threshold, grip_release_threshold);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] black box recording: %s",
                arm_name.c_str(), blackbox.is_open() ? "on" : "off");

    rclcpp::spin(node);

    // 等后台重锚定结束再碰 SDK，避免 servo_move_enable/logout 与它竞争。
    if (rearm_thread.joinable())
        rearm_thread.join();

    // 优雅退出。只有真正连上过（且不是 dry-run）才碰臂：对断开连接调用
    // servo_move_enable/login_out 会阻塞到 SDK 超时，导致 Ctrl-C 挂死；被 SIGKILL
    // 又会在控制器上留下会话，挡住下一次 login。
    if (arm_ok && !dry_run)
    {
        if (gripper != nullptr && gripper->closed())
        {
            std::string error;
            if (!gripper->set_closed(false, &error))
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] failed to open PGI gripper on shutdown: %s",
                            arm_name.c_str(), error.c_str());
            }
        }
        robot.servo_move_enable(false);
        robot.login_out();
    }
    if (blackbox.is_open())
    {
        blackbox.flush();
        blackbox.close();
    }
    rclcpp::shutdown();
    return 0;
}
