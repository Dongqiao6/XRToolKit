"""SRP 与 ROS 的单位/位姿转换（从 demo utils/ros_bridge.py 复制并去掉三方依赖）。

SRP 的位姿与角度单位是毫米/毫弧度，速度是 mm/s、mrad/s；ROS 用 m、rad。
Euler → 四元数沿用 transforms3d ``euler2quat(roll, pitch, yaw, 'sxyz')`` 的
标准 roll-pitch-yaw 语义，直接展开，避免运行期依赖 transforms3d。
"""

import math

from geometry_msgs.msg import PoseStamped, Quaternion


def srp_to_ros_unit(n: float) -> float:
    return n / 1000.0


def ros_to_srp_unit(n: float) -> float:
    return n * 1000.0


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """返回 ``(w, x, y, z)``；与 transforms3d 的 sxyz 顺序一致。"""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def to_ros_pose(protobuf_pose) -> PoseStamped:
    """SRP ``Pose`` → ``geometry_msgs/PoseStamped``（单位换算后）。"""
    ros_pose = PoseStamped()
    ros_pose.pose.position.x = srp_to_ros_unit(protobuf_pose.x)
    ros_pose.pose.position.y = srp_to_ros_unit(protobuf_pose.y)
    ros_pose.pose.position.z = srp_to_ros_unit(protobuf_pose.z)
    quat = euler_to_quaternion(
        srp_to_ros_unit(protobuf_pose.roll),
        srp_to_ros_unit(protobuf_pose.pitch),
        srp_to_ros_unit(protobuf_pose.yaw),
    )
    ros_pose.pose.orientation.w = quat[0]
    ros_pose.pose.orientation.x = quat[1]
    ros_pose.pose.orientation.y = quat[2]
    ros_pose.pose.orientation.z = quat[3]
    return ros_pose


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
