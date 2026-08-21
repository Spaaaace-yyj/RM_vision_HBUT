#pragma once

// BuffDetectorNode：能量机关自动瞄准全链路节点。
// 订阅图像 → 检测 → 筛选 → PnP → tf 变换 → 跟踪 → 预测 → 发布 /tracker/target。
// 实车接相机图像，离线用 video_pub 回放视频。

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core.hpp>

#include "buff_algo/locate/buff_pnp_solver.hpp"
#include "buff_algo/predictor/buff_predictor.hpp"
#include "buff_algo/selector/buff_selector.hpp"
#include "buff_algo/tracker/buff_tracker.hpp"
#include "buff_detector/onnx_buff_detector.hpp"

namespace buff_auto_aim {

class BuffDetectorNode : public rclcpp::Node {
public:
    explicit BuffDetectorNode(const rclcpp::NodeOptions& options);

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

    // 相机系 Pose3f 变换到 target_frame_，跟踪器要求在静止系工作
    bool transformPose(
        const buff_algo::Pose3f& in, buff_algo::Pose3f& out,
        const rclcpp::Time& stamp);

    // ROS2 参数拼成 BuffTracker 构造所需的 YAML 字符串
    // 不能是 const，内部要调 declare_parameter
    std::string make_tracker_yaml();

    // ---- 订阅 / 发布 ----
    std::shared_ptr<image_transport::Subscriber> img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
    std::shared_ptr<image_transport::Publisher> debug_pub_;

    // ---- tf ----
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    // ---- 算法模块 ----
    std::unique_ptr<buff_detector::OnnxBuffDetector> detector_;
    std::unique_ptr<buff_algo::BuffSelector> selector_;
    std::unique_ptr<buff_algo::BuffPnPSolver> pnp_solver_;
    std::unique_ptr<buff_algo::BuffTracker> tracker_;
    std::unique_ptr<buff_algo::BuffPredictor> predictor_;

    // ---- 参数 ----
    std::string target_frame_;
    std::string camera_frame_;
    int buff_mode_ = 1;    // 1=小符 2=大符
    int buff_color_ = 1;   // 蓝队打红符
    double buff_radius_ = 0.7;  // R 中心到打击点的距离，实车按机械尺寸改
    bool debug_image_ = false;
    bool has_camera_info_ = false;
    cv::Mat camera_matrix_;
    cv::Mat distortion_;

    // video_pub 回放的图像没有时间戳，用节点时钟兜底
    rclcpp::Time last_image_time_;
};

}  // namespace buff_auto_aim
