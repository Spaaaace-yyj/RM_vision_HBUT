// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

// STD
#include <atomic>
#include <cstddef>
#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/detector_throughput.hpp"
#include "armor_detector/embedded_camera.hpp"
#include "armor_detector/neural_detector.hpp"
#include "armor_detector/openvino_detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/debug_detector_timing.hpp"
#include "auto_aim_interfaces/msg/debug_detector_fps.hpp"

namespace rm_auto_aim
{
    class ArmorDetectorNode : public rclcpp::Node
    {
    public:
        ArmorDetectorNode(const rclcpp::NodeOptions& options);
        ~ArmorDetectorNode() override;

    private:
        struct CaptureTiming
        {
            bool embedded = false;
            std::chrono::steady_clock::time_point ready;
            double wait_ms = 0.0, process_ms = 0.0, info_publish_ms = 0.0;
        };
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
        void processImage(
            const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, const CaptureTiming & capture);
        void publishFrameTiming(bool completed);
        void publishThroughput();
        void markFrameFailure(const std::string & error);
        void setNeuralTiming(const NeuralDetector::Timing & timing);
        std::vector<Armor> runTraditional(const cv::Mat & img);
        void initEmbeddedCamera();
        void captureLoop();

        void completeFrame(
            const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, std::vector<Armor> armors,
            std::chrono::steady_clock::time_point callback_start, bool neural_frame);
        bool ensureOpenVINODetector();
        bool submitOpenVINO(
            const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
            std::chrono::steady_clock::time_point callback_start);
        void drainOpenVINOResults();
        void updateNeuralParams();

        std::unique_ptr<Detector> initDetector();
        std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

        // 神经网络模式相关（detector_mode = neural 时才会用到）
        void initNeuralParams();
        bool setDetectorMode(const std::string& mode);
        bool ensureNeuralDetector();
        std::vector<Armor> detectArmorsByNeural(const cv::Mat& img, bool& traditional_ran);
        void publishNeuralDebugImage(
            const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const std::vector<Armor>& armors);

        void createDebugPublishers();
        void destroyDebugPublishers();

        void publishMarkers();

        //tf
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Armor Detector（传统视觉模式，保持原有实现不动）
        std::unique_ptr<Detector> detector_;

        // 检测模式：traditional=传统识别（默认，行为与以前完全一致），neural=神经网络模式。
        // 模式标志与 generation 过滤在途旧帧；节点共享状态由 processing_mutex_ 串行保护。
        std::atomic<bool> neural_mode_{false};
        std::string detector_mode_str_ = "traditional";
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mode_cb_handle_;
        // 神经网络检测器，切到 neural 模式后第一次用到时才加载模型
        std::unique_ptr<NeuralDetector> neural_detector_;
        std::atomic<bool> neural_load_failed_{false};
        NeuralDetectorParams neural_params_;
        std::string neural_model_path_;
        // 本帧是否跑过传统流程（仅模型不可用/推理异常时安全退回）
        bool traditional_ran_ = false;
        bool neural_frame_succeeded_ = false;

        auto_aim_interfaces::msg::DebugDetectorTiming frame_timing_;
        std::chrono::steady_clock::time_point frame_entered_, frame_ready_;
        DetectorThroughput throughput_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugDetectorTiming>::SharedPtr timing_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugDetectorFps>::SharedPtr fps_pub_;
        rclcpp::TimerBase::SharedPtr fps_timer_;
        struct PendingNeuralFrame
        {
            sensor_msgs::msg::Image::ConstSharedPtr image;
            auto_aim_interfaces::msg::DebugDetectorTiming timing;
            std::chrono::steady_clock::time_point started, entered, ready;
            std::uint64_t generation;
        };
        std::string neural_backend_ = "onnxruntime";
        OpenVINOConfig openvino_config_;
        std::unique_ptr<OpenVINODetector> openvino_detector_;
        rclcpp::TimerBase::SharedPtr openvino_timer_;
        std::unordered_map<std::uint64_t, PendingNeuralFrame> pending_neural_frames_;
        std::atomic<std::uint64_t> neural_generation_{0};
        std::uint64_t next_frame_id_ = 0;
        std::uint64_t openvino_rejected_ = 0;
        NeuralDetector::Timing last_openvino_timing_;


        // Detected armors publisher
        auto_aim_interfaces::msg::Armors armors_msg_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

        // Visualization marker publisher
        visualization_msgs::msg::Marker armor_marker_;
        visualization_msgs::msg::Marker text_marker_;
        visualization_msgs::msg::MarkerArray marker_array_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        // Camera info part
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        cv::Point2f cam_center_;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

        std::unique_ptr<PnPSolver> pnp_solver_;

        // Image subscrpition
        std::shared_ptr<image_transport::Subscriber> img_sub_;

        // main's direct SDK input, with independent frame ownership for async inference.
        bool use_embedded_camera_ = false;
        std::string camera_frame_id_;
        std::unique_ptr<EmbeddedCamera> embedded_camera_;
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
        std::atomic<bool> camera_running_{false};
        std::thread capture_thread_;
        // Protect node state shared by SDK capture, result timer and parameter callbacks.
        // OpenVINO inference itself remains concurrent inside its own request pool.
        std::mutex processing_mutex_;

        // Debug information
        bool debug_;
        std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
        image_transport::Publisher binary_img_pub_;
        image_transport::Publisher number_img_pub_;
        image_transport::Publisher result_img_pub_;

        // 录制视频
        bool is_record_;
        cv::VideoWriter video_writer_;
    };
} // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
