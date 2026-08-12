#pragma once

// buff_algo 公共数据类型
//
// 本库不依赖 ROS2 / tf2 / 自定义消息类型，只依赖 Eigen 与 OpenCV。
// 如果需要在 ROS2 节点中使用，请在调用处把 ROS 消息 / tf2 类型转换为下面的
// POD 结构体（转换代码可参考主仓库中 autoaim_* 包内的适配层实现）。

#include <array>
#include <eigen3/Eigen/Dense>

namespace buff_algo {

// 一个刚体位姿：位置 + 姿态四元数。
// 对应 ROS 里的 tf2::Transform / geometry_msgs::msg::Pose。
struct Pose3f {
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
};

// 能量机关目标检测结果（对应神经网络+PnP前置的4个角点检测）。
// 角点顺序固定为 上(t) 左(l) 下(b) 右(r)，与主仓库
// autoaim_interfaces::msg::BuffDetection 保持一致。
struct BuffDetection {
    int color = -1;
    int label = -1;
    float confidence = 0.0f;

    Eigen::Vector2f t = Eigen::Vector2f::Zero();
    Eigen::Vector2f l = Eigen::Vector2f::Zero();
    Eigen::Vector2f b = Eigen::Vector2f::Zero();
    Eigen::Vector2f r = Eigen::Vector2f::Zero();
};

// 能量机关运动状态估计结果（对应主仓库 autoaim_interfaces::msg::BuffState）。
struct BuffState {
    Eigen::Vector3f r_center = Eigen::Vector3f::Zero();

    float yaw = 0.0f;
    float yaw_velocity = 0.0f;

    float roll = 0.0f;
    float roll_velocity = 0.0f;

    // 大符正弦模型参数： roll_velocity(t) = a * sin(w * t + phi) + b
    float phi = 0.0f;
    float a = 0.0f;
    float w = 0.0f;
    float b = 0.0f;

    // 1: small buff, 2: big buff
    uint8_t mode = 0;
};

// 能量机关的工作模式。
enum class BuffMode : int {
    NONE = 0,
    SMALL_BUFF = 1,
    BIG_BUFF = 2,
};

// 检测模型使用的能量机关颜色编号。
enum class BuffColor : int {
    BLUE = 0,
    RED = 1,
};

// 能量机关扇叶激活状态。
enum class BuffActivation : int {
    NONE = -1,
    INACTIVATE = 0,
    ACTIVATE = 1,
};

namespace consts {

// 能量机关 R 字中心到扇叶打击点的半径，单位：米
inline constexpr float BUFF_RADIUS = 0.7f;
// 小符固定角速度，单位：rad/s
inline constexpr float SMALL_BUFF_PALSTANCE = 1.047197551f;
// 能量机关扇叶面板边长的一半，单位：米
inline constexpr float BUFF_WIDTH = 0.114f;

// 能量机关扇面 4 个角点的物体坐标（PnP用），顺序为 上/左/下/右，
// 坐标系：前x，左y，上z（与主仓库装甲板坐标系一致）。
inline const std::array<Eigen::Vector3f, 4> BUFF_POINTS = {
    Eigen::Vector3f(0, 0, BUFF_WIDTH),
    Eigen::Vector3f(0, BUFF_WIDTH, 0),
    Eigen::Vector3f(0, 0, -BUFF_WIDTH),
    Eigen::Vector3f(0, -BUFF_WIDTH, 0),
};

} // namespace consts

} // namespace buff_algo
