#include "buff_detector/buff_detector_node.hpp"

#include <cmath>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/imgproc.hpp>

#include "buff_algo/common/math_utils.hpp"  // r2d

namespace buff_auto_aim {

BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("buff_detector_node", options),
      tf2_buffer_(this->get_clock()),
      tf2_listener_(tf2_buffer_) {
    // ---- 参数 ----
    const std::string model_path = this->declare_parameter("model_path", "buff.onnx");
    // 模型格式：zju=浙大原版，szu=深大 5 点模型（CPU 推理约快一倍）
    const std::string detector_format = this->declare_parameter("detector_format", "zju");
    const float confidence_threshold =
        static_cast<float>(this->declare_parameter("confidence_threshold", 0.5));
    const float nms_threshold =
        static_cast<float>(this->declare_parameter("nms_threshold", 0.4));
    buff_mode_ = this->declare_parameter("buff_mode", 2);   // 默认大符
    buff_color_ = this->declare_parameter("buff_color", 1); // 蓝队打红符
    buff_radius_ = this->declare_parameter("buff_radius", 0.7);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    camera_frame_ = this->declare_parameter("camera_frame", "camera_optical_frame");
    debug_image_ = this->declare_parameter("debug_image", false);
    const bool subscribe_compressed = this->declare_parameter("subscribe_compressed", false);

    // ---- 算法模块 ----
    const buff_detector::DetectorFormat format =
        detector_format == "szu" ? buff_detector::DetectorFormat::SZU
                                 : buff_detector::DetectorFormat::ZJU;
    detector_ = std::make_unique<buff_detector::OnnxBuffDetector>(
        model_path, buff_algo::BuffDetectorConfig{confidence_threshold, nms_threshold}, format);

    selector_ = std::make_unique<buff_algo::BuffSelector>(buff_color_);

    pnp_solver_ = std::make_unique<buff_algo::BuffPnPSolver>();

    cv::FileStorage fs(
        make_tracker_yaml(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
    tracker_ = std::make_unique<buff_algo::BuffTracker>(fs["buff_tracker"]);
    tracker_->set_mode(static_cast<buff_algo::BuffMode>(buff_mode_));

    cv::FileStorage predictor_fs(
        "%YAML:1.0\n---\nbuff_predictor: {}\n",
        cv::FileStorage::READ | cv::FileStorage::MEMORY);
    predictor_ = std::make_unique<buff_algo::BuffPredictor>(
        predictor_fs["buff_predictor"]);

    // ---- 订阅 ----
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            cameraInfoCallback(msg);
        });

    const std::string transport = subscribe_compressed ? "compressed" : "raw";
    img_sub_ = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
            this, "/image_raw",
            std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1),
            transport, rmw_qos_profile_sensor_data));

    // ---- 发布 ----
    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
        "/tracker/target", rclcpp::SensorDataQoS());
    if (debug_image_) {
        debug_pub_ = std::make_shared<image_transport::Publisher>(
            image_transport::create_publisher(this, "/buff/debug_image"));
    }
}

void BuffDetectorNode::cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (!has_camera_info_) {
        camera_matrix_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
                                                    msg->k[3], msg->k[4], msg->k[5],
                                                    msg->k[6], msg->k[7], msg->k[8]);
        distortion_ = cv::Mat(1, 8, CV_64F, const_cast<double*>(msg->d.data())).clone();
        selector_->set_camera_matrix(camera_matrix_, distortion_);
        pnp_solver_->set_camera_matrix(camera_matrix_, distortion_);
        has_camera_info_ = true;
        RCLCPP_INFO(get_logger(), "camera info received, fx=%.1f", msg->k[0]);
    }
}

void BuffDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // 必须解码成 BGR：检测器内部会做一次 RGB 交换，若解码成 rgb8 会双重交换，
    // 网络实际收到 BGR 图像，颜色类别全部判错
    const auto img = cv_bridge::toCvShare(msg, "bgr8")->image;

    // 回放图像没有时间戳时用节点时钟兜底
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const bool has_stamp = timestamp >= 1e-6;
    if (!has_stamp) {
        timestamp = this->now().seconds();
    }
    // 用图像采集时刻查 tf，云台转的时候相机位姿才是当时的
    const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();

    // 1. 检测
    const std::vector<buff_algo::DetectedBuff> detected =
        detector_->detect_with_keypoints(img);
    std::vector<buff_algo::BuffDetection> detections;
    detections.reserve(detected.size());
    for (const auto& item : detected) detections.push_back(item.detection);

    // 2. 筛选 + PnP，需要相机内参就绪
    std::vector<buff_algo::Pose3f> poses;
    if (has_camera_info_) {
        std::vector<buff_algo::BuffDetection> selected;
        selector_->select_buffs(detections, selected);
        poses = pnp_solver_->solve_pnp(selected);

        // 过滤异常位姿，NaN 会永久污染跟踪器
        poses.erase(
            std::remove_if(
                poses.begin(), poses.end(),
                [](const buff_algo::Pose3f& p) {
                    return !p.translation.allFinite() ||
                           !p.rotation.coeffs().allFinite();
                }),
            poses.end());
    }

    // 4. 变换到静止参考系，否则分不清目标运动和云台运动
    for (auto& pose : poses) {
        buff_algo::Pose3f transformed;
        if (transformPose(pose, transformed, stamp)) {
            pose = transformed;
        } else {
            poses.clear();
            break;
        }
    }

    // 5. 跟踪
    if (!poses.empty()) {
        tracker_->push(buff_algo::Buff(poses[0]));
    }
    tracker_->update(timestamp);

    // 6. 预测当前时刻打击点，弹道提前量由 gimbal_controller 统一做。
    //    打击点是扇叶上距 R 中心 BUFF_RADIUS 处的装甲模块位置，
    //    R 字只是标志，打 R 中心无效。
    const buff_algo::BuffState state = tracker_->get_state();
    predictor_->set_state(state, timestamp, timestamp);
    Eigen::Vector3f aim_point = predictor_->predict_position(0.0f);
    // 实车 R 中心到打击点的距离和浙大写死的 0.7m 可能不同，
    // 按 buff_radius 参数等比缩放偏移量，默认 0.7 与原行为一致
    if (buff_radius_ != buff_algo::consts::BUFF_RADIUS) {
        const float k = static_cast<float>(buff_radius_ / buff_algo::consts::BUFF_RADIUS);
        aim_point = state.r_center + (aim_point - state.r_center) * k;
    }

    // 6. 发布目标消息，gimbal_controller 可以直接使用
    auto_aim_interfaces::msg::Target target_msg;
    target_msg.header.stamp = msg->header.stamp;
    target_msg.header.frame_id = target_frame_;
    // TEMP_LOST 时跟踪器还在往外推，预测点依然有效，和装甲板链路的习惯一致
    const bool tracking =
        tracker_->status() == buff_algo::StatusType::TRACKING ||
        tracker_->status() == buff_algo::StatusType::TEMP_LOST;
    target_msg.tracking = tracking;
    target_msg.id = "buff";
    // armors_num 置 0 表示丢失，云台按丢目标处理；
    // 发 1 加零位置会把云台引到原点
    target_msg.armors_num = tracking ? 1 : 0;
    target_msg.type = "buff";
    if (tracking) {
        target_msg.position.x = aim_point.x();
        target_msg.position.y = aim_point.y();
        target_msg.position.z = aim_point.z();
        // 打击点的切向线速度 (0, R*sin(roll)*ω, R*cos(roll)*ω)
        const float omega = state.roll_velocity;
        const float radius = buff_algo::consts::BUFF_RADIUS;
        target_msg.velocity.x = 0.0;
        target_msg.velocity.y = radius * std::sin(state.roll) * omega;
        target_msg.velocity.z = radius * std::cos(state.roll) * omega;
        target_msg.yaw = state.roll;
        target_msg.v_yaw = state.roll_velocity;
    }
    // 丢了就清零，别把 (0, -R, 0) 这种垃圾点发给云台
    target_msg.radius_1 = 0.0;
    target_msg.radius_2 = 0.0;
    target_msg.dz = 0.0;
    target_pub_->publish(target_msg);

    // 调试：状态变化时打印
    static buff_algo::StatusType last_status = buff_algo::StatusType::LOST;
    if (tracker_->status() != last_status) {
        RCLCPP_INFO(
            get_logger(), "status: %s | mode: %d | roll: %.1f° | v_roll: %.1f°/s | aim=(%.2f, %.2f, %.2f)",
            tracker_->status() == buff_algo::StatusType::TRACKING   ? "TRACKING" :
            tracker_->status() == buff_algo::StatusType::CONVERGING ? "CONVERGING" :
            tracker_->status() == buff_algo::StatusType::TEMP_LOST  ? "TEMP_LOST" :
                                                                      "LOST",
            static_cast<int>(state.mode),
            buff_algo::r2d(state.roll), buff_algo::r2d(state.roll_velocity),
            aim_point.x(), aim_point.y(), aim_point.z());
        last_status = tracker_->status();
    }

    // 调试图像：画关键点和预测点
    if (debug_image_) {
        cv::Mat debug_img = img.clone();
        for (const auto& item : detected) {
            for (std::size_t i = 0; i < item.keypoints.size(); ++i) {
                cv::circle(
                    debug_img,
                    {cvRound(item.keypoints[i].x()), cvRound(item.keypoints[i].y())},
                    4, cv::Scalar(0, 255, 0), cv::FILLED);
            }
        }
        if (has_camera_info_ && aim_point.x() > 0.1f) {
            // 相机系「前x 左y 上z」→ 像素: u = cx + fx*(-y)/x, v = cy + fy*(-z)/x
            const double fx = camera_matrix_.at<double>(0, 0);
            const double fy = camera_matrix_.at<double>(1, 1);
            const double cx = camera_matrix_.at<double>(0, 2);
            const double cy = camera_matrix_.at<double>(1, 2);
            const int px = cvRound(-aim_point.y() / aim_point.x() * fx + cx);
            const int py = cvRound(-aim_point.z() / aim_point.x() * fy + cy);
            cv::circle(debug_img, {px, py}, 12, cv::Scalar(0, 255, 255), 3);
        }
        debug_pub_->publish(cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg());
    }
}

bool BuffDetectorNode::transformPose(
    const buff_algo::Pose3f& in, buff_algo::Pose3f& out,
    const rclcpp::Time& stamp) {
    // 目标系和相机系相同时跳过变换，用于离线调试
    if (target_frame_ == camera_frame_) {
        out = in;
        return true;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = camera_frame_;
    ps.header.stamp = stamp;
    ps.pose.position.x = in.translation.x();
    ps.pose.position.y = in.translation.y();
    ps.pose.position.z = in.translation.z();
    ps.pose.orientation.w = in.rotation.w();
    ps.pose.orientation.x = in.rotation.x();
    ps.pose.orientation.y = in.rotation.y();
    ps.pose.orientation.z = in.rotation.z();

    try {
        const auto transformed = tf2_buffer_.transform(ps, target_frame_);
        out.translation = Eigen::Vector3f(
            transformed.pose.position.x, transformed.pose.position.y,
            transformed.pose.position.z);
        out.rotation = Eigen::Quaternionf(
            transformed.pose.orientation.w, transformed.pose.orientation.x,
            transformed.pose.orientation.y, transformed.pose.orientation.z);
        return true;
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "tf transform failed: %s", error.what());
        return false;
    }
}

std::string BuffDetectorNode::make_tracker_yaml() {
    // 把 ROS2 参数拼成 BuffTracker 构造所需的 FileStorage YAML
    const float switch_buff_angle =
        static_cast<float>(this->declare_parameter("switch_buff_angle", 45.8));
    const float r_center_ratio =
        static_cast<float>(this->declare_parameter("R_center_filter_ratio", 0.1));
    const float q_pos = static_cast<float>(this->declare_parameter("kf_q_pos", 1e-1));
    const float q_vel = static_cast<float>(this->declare_parameter("kf_q_vel", 1e+0));
    const float r_meas = static_cast<float>(this->declare_parameter("kf_r_meas", 1e-1));
    const int small_temp_lost =
        this->declare_parameter("small_max_temp_lost_frames", 20);
    const int small_converging =
        this->declare_parameter("small_max_converging_frames", 20);
    const int big_temp_lost =
        this->declare_parameter("big_max_temp_lost_frames", 20);
    const int big_converging =
        this->declare_parameter("big_max_converging_frames", 300);

    std::ostringstream oss;
    oss << "%YAML:1.0\n---\nbuff_tracker:\n";
    oss << "  small_buff_observer:\n";
    oss << "    switch_buff_angle: " << switch_buff_angle << "\n";
    oss << "    R_center_filter_ratio: " << r_center_ratio << "\n";
    oss << "    kf_roll:\n";
    oss << "      process_noise: !!opencv-matrix\n";
    oss << "        rows: 2\n        cols: 2\n        dt: f\n";
    oss << "        data: [ " << q_pos << ", 0, 0, " << q_vel << " ]\n";
    oss << "      measurement_noise: !!opencv-matrix\n";
    oss << "        rows: 1\n        cols: 1\n        dt: f\n";
    oss << "        data: [ " << r_meas << " ]\n";
    oss << "    kf_yaw:\n";
    oss << "      process_noise: !!opencv-matrix\n";
    oss << "        rows: 2\n        cols: 2\n        dt: f\n";
    oss << "        data: [ " << q_pos << ", 0, 0, " << q_vel << " ]\n";
    oss << "      measurement_noise: !!opencv-matrix\n";
    oss << "        rows: 1\n        cols: 1\n        dt: f\n";
    oss << "        data: [ " << r_meas << " ]\n";
    oss << "  big_buff_observer:\n";
    oss << "    switch_buff_angle: " << switch_buff_angle << "\n";
    oss << "    R_center_filter_ratio: " << r_center_ratio << "\n";
    oss << "    ransac:\n";
    oss << "      max_iterations: " << this->declare_parameter("ransac_max_iterations", 200) << "\n";
    oss << "      threshold: " << this->declare_parameter("ransac_threshold", 0.5) << "\n";
    oss << "      min_omega: " << this->declare_parameter("ransac_min_omega", 1.884) << "\n";
    oss << "      max_omega: " << this->declare_parameter("ransac_max_omega", 2.0) << "\n";
    oss << "      min_amplitude: " << this->declare_parameter("ransac_min_amplitude", 0.78) << "\n";
    oss << "      max_amplitude: " << this->declare_parameter("ransac_max_amplitude", 1.045) << "\n";
    oss << "      min_inliers: " << this->declare_parameter("ransac_min_inliers", 100) << "\n";
    oss << "      max_abs_speed: " << this->declare_parameter("ransac_max_abs_speed", 2.09) << "\n";
    oss << "      fit_interval: " << this->declare_parameter("ransac_fit_interval", 5) << "\n";
    oss << "    kf_yaw:\n";
    oss << "      process_noise: !!opencv-matrix\n";
    oss << "        rows: 2\n        cols: 2\n        dt: f\n";
    oss << "        data: [ " << q_pos << ", 0, 0, " << q_vel << " ]\n";
    oss << "      measurement_noise: !!opencv-matrix\n";
    oss << "        rows: 1\n        cols: 1\n        dt: f\n";
    oss << "        data: [ " << r_meas << " ]\n";
    oss << "  small_buff_status:\n";
    oss << "    max_temp_lost_frames: " << small_temp_lost << "\n";
    oss << "    max_converging_frames: " << small_converging << "\n";
    oss << "  big_buff_status:\n";
    oss << "    max_temp_lost_frames: " << big_temp_lost << "\n";
    oss << "    max_converging_frames: " << big_converging << "\n";
    oss << "  temp_lost_return_frames: 5\n";
    oss << "  err_queue_size: 0\n";
    oss << "  approximate_framerate: 0\n";
    return oss.str();
}

}  // namespace buff_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(buff_auto_aim::BuffDetectorNode)
