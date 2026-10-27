// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <iomanip>
#include <map>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/neural_postprocessor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
    namespace
    {
        using SteadyClock = std::chrono::steady_clock;
        double elapsedMs(SteadyClock::time_point start)
        {
            return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
        }
    }  // namespace

    ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
        : Node("armor_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

        // Detector
        detector_ = initDetector();

        // 神经网络模式的参数，默认还是传统模式，行为保持不变
        initNeuralParams();

        // Armors Publisher
        armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
            "/detector/armors", rclcpp::SensorDataQoS());

        timing_pub_ = create_publisher<auto_aim_interfaces::msg::DebugDetectorTiming>(
            "/detector/debug_timing", rclcpp::SensorDataQoS());
        fps_pub_ = create_publisher<auto_aim_interfaces::msg::DebugDetectorFps>(
            "/detector/debug_fps", rclcpp::SensorDataQoS());
        fps_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishThroughput(); });

        // Visualization Marker Publisher
        // See http://wiki.ros.org/rviz/DisplayTypes/Marker
        armor_marker_.ns = "armors";
        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.scale.x = 0.05;
        armor_marker_.scale.z = 0.125;
        armor_marker_.color.a = 1.0;
        armor_marker_.color.g = 0.5;
        armor_marker_.color.b = 1.0;
        armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        text_marker_.ns = "classification";
        text_marker_.action = visualization_msgs::msg::Marker::ADD;
        text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker_.scale.z = 0.1;
        text_marker_.color.a = 1.0;
        text_marker_.color.r = 1.0;
        text_marker_.color.g = 1.0;
        text_marker_.color.b = 1.0;
        text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

        // Debug Publishers
        debug_ = this->declare_parameter("debug", false);
        if (debug_)
        {
            createDebugPublishers();
        }

        // Debug param change moniter
        debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        debug_cb_handle_ =
            debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p)
            {
                const std::lock_guard<std::mutex> lock(processing_mutex_);
                debug_ = p.as_bool();
                debug_ ? createDebugPublishers() : destroyDebugPublishers();
            });

        // 检测模式可以在线切换：
        //   ros2 param set /armor_detector detector_mode neural
        //   ros2 param set /armor_detector detector_mode traditional
        // 事件回调与采集/结果回调共用 processing_mutex_；模型仍在处理帧时懒加载。
        mode_cb_handle_ = debug_param_sub_->add_parameter_callback(
            "detector_mode", [this](const rclcpp::Parameter& p)
            {
                const std::lock_guard<std::mutex> lock(processing_mutex_);
                setDetectorMode(p.as_string());
            });

        //tf
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rcl_interfaces::msg::ParameterDescriptor source_desc;
        source_desc.read_only = true;
        use_embedded_camera_ = this->declare_parameter("use_embedded_camera", false, source_desc);
        if (use_embedded_camera_)
        {
            initEmbeddedCamera();
        }
        else
        {
            cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_info", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
                {
                    const std::lock_guard<std::mutex> lock(processing_mutex_);
                    cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                    pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d, tf_buffer_);
                    cam_info_sub_.reset();
                });

            std::string transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
            auto qos_image_sub = rmw_qos_profile_sensor_data;
            qos_image_sub.depth = 1;
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                this, "/image_raw", std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1),
                transport_, qos_image_sub));
        }

        // 用opencv录制视频,图像类型为rgb8
        is_record_ = this->declare_parameter("is_record", false);
        if (is_record_)
        {
            std::string save_video_path = this->declare_parameter("save_video_path", "armor.avi");
            int save_video_fps = this->declare_parameter("save_video_fps", 30);
            int save_video_width = this->declare_parameter("save_video_width", 640);
            int save_video_height = this->declare_parameter("save_video_height", 480);
            video_writer_.open(
                save_video_path, cv::VideoWriter::fourcc('P', 'I', 'M', '1'), save_video_fps,
                cv::Size(save_video_width, save_video_height), true);
        }
        if (neural_backend_ == "openvino" && OpenVINODetector::available())
        {
            openvino_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(2), [this]()
                {
                    const std::lock_guard<std::mutex> lock(processing_mutex_);
                    drainOpenVINOResults();
                });
        }
        // Start last: camera callbacks must not observe partly initialized node state.
        if (use_embedded_camera_)
        {
            camera_running_ = true;
            capture_thread_ = std::thread([this]() { captureLoop(); });
        }
    }

    ArmorDetectorNode::~ArmorDetectorNode()
    {
        camera_running_ = false;
        if (capture_thread_.joinable()) { capture_thread_.join(); }
        embedded_camera_.reset();
        img_sub_.reset();
        if (fps_timer_) { fps_timer_->cancel(); }
        fps_timer_.reset();
        if (openvino_timer_) { openvino_timer_->cancel(); }
        openvino_timer_.reset();
        openvino_detector_.reset();  // Join inference callbacks while all node resources still exist.
    }

    void ArmorDetectorNode::initEmbeddedCamera()
    {
        // Reuse /mv_camera settings supplied by armor_embedded_launch.py.
        // These describe SDK startup state; changes require restarting the node.
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.read_only = true;
        EmbeddedCameraParams params;
        params.exposure_time = declare_parameter("exposure_time", 5000, desc);
        params.analog_gain = declare_parameter("analog_gain", -1, desc);
        params.r_gain = declare_parameter("rgb_gain.r", -1, desc);
        params.g_gain = declare_parameter("rgb_gain.g", -1, desc);
        params.b_gain = declare_parameter("rgb_gain.b", -1, desc);
        params.gamma = declare_parameter("gamma", -1, desc);
        params.saturation = declare_parameter("saturation", -1, desc);
        params.flip_image = declare_parameter("flip_image", false, desc);
        camera_frame_id_ = declare_parameter("frame_id", std::string("camera_optical_frame"), desc);
        const auto camera_name = declare_parameter("camera_name", std::string("mv_camera"), desc);
        const auto info_url = declare_parameter(
            "camera_info_url", std::string("package://bringup/config/camera_info.yaml"), desc);
        camera_info_manager_ =
            std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);
        if (!camera_info_manager_->validateURL(info_url) || !camera_info_manager_->loadCameraInfo(info_url))
        {
            throw std::runtime_error("Embedded camera calibration cannot be loaded: " + info_url);
        }
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_manager_->getCameraInfo());
        if (cam_info_->k[0] <= 0.0 || cam_info_->k[4] <= 0.0)
        {
            throw std::runtime_error("Embedded camera calibration has invalid focal lengths: " + info_url);
        }
        cam_info_->header.frame_id = camera_frame_id_;
        cam_center_ = cv::Point2f(cam_info_->k[2], cam_info_->k[5]);
        pnp_solver_ = std::make_unique<PnPSolver>(cam_info_->k, cam_info_->d, tf_buffer_);
        cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", rclcpp::SensorDataQoS());
        embedded_camera_ = std::make_unique<EmbeddedCamera>(params);
        RCLCPP_INFO(get_logger(),
            "Embedded MindVision camera: SDK -> detector, no /image_raw subscription; "
            "calibration=%s (fx=%.1f fy=%.1f cx=%.1f cy=%.1f)",
            info_url.c_str(), cam_info_->k[0], cam_info_->k[4], cam_info_->k[2], cam_info_->k[5]);
    }

    void ArmorDetectorNode::captureLoop()
    {
        sensor_msgs::msg::Image::SharedPtr img_msg;
        while (rclcpp::ok() && camera_running_.load())
        {
            try
            {
                // A pending OpenVINO result retains its source image/header for PnP and debug.
                // Reuse storage only when no in-flight frame still owns it (main was synchronous).
                if (!img_msg || img_msg.use_count() != 1)
                {
                    img_msg = std::make_shared<sensor_msgs::msg::Image>();
                    img_msg->encoding = "rgb8";
                    img_msg->is_bigendian = 0;
                    img_msg->header.frame_id = camera_frame_id_;
                }
                CaptureTiming capture;
                capture.embedded = true;
                const auto read_started = SteadyClock::now();
                auto acquired = read_started;
                const bool received = embedded_camera_->read(
                    img_msg->data, img_msg->width, img_msg->height,
                    [this, &img_msg, &acquired]()
                    {
                        acquired = SteadyClock::now();
                        img_msg->header.stamp = now();
                    });
                capture.ready = SteadyClock::now();
                capture.wait_ms = std::chrono::duration<double, std::milli>(acquired - read_started).count();
                capture.process_ms = std::chrono::duration<double, std::milli>(capture.ready - acquired).count();
                if (!camera_running_.load()) { break; }
                if (!received)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Embedded camera frame timeout");
                    continue;
                }
                img_msg->step = img_msg->width * 3;
                // Publish a copy; the calibration stored by the node remains immutable.
                auto camera_info = *cam_info_;
                camera_info.header = img_msg->header;
                const auto info_started = SteadyClock::now();
                cam_info_pub_->publish(camera_info);
                capture.info_publish_ms = elapsedMs(info_started);
                // SDK buffer is already released. This never waits for async inference.
                processImage(img_msg, capture);
            }
            catch (const std::exception & error)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Embedded camera/detector frame error: %s", error.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        processImage(img_msg, CaptureTiming{});
    }

    void ArmorDetectorNode::processImage(
        const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, const CaptureTiming & capture)
    {
        const auto entered = SteadyClock::now();
        const auto entered_ros = now();
        const std::lock_guard<std::mutex> lock(processing_mutex_);
        const double mutex_wait_ms = elapsedMs(entered);
        throughput_.add(DetectorThroughput::INPUT);
        const auto drain_started = SteadyClock::now();
        drainOpenVINOResults();
        const double drain_ms = elapsedMs(drain_started);
        const auto callback_start = SteadyClock::now();
        const bool neural_frame = neural_mode_.load();
        frame_timing_ = auto_aim_interfaces::msg::DebugDetectorTiming();
        frame_timing_.header = img_msg->header;
        frame_timing_.frame_id = next_frame_id_++;
        frame_timing_.detector_mode = neural_frame ? "neural" : "traditional";
        frame_timing_.neural_backend = neural_frame ? neural_backend_ : "";
        frame_timing_.embedded_camera = capture.embedded;
        frame_timing_.camera_wait_ms = capture.wait_ms;
        frame_timing_.camera_process_ms = capture.process_ms;
        frame_timing_.camera_info_publish_ms = capture.info_publish_ms;
        frame_timing_.input_age_ms = (entered_ros - img_msg->header.stamp).seconds() * 1000.0;
        frame_timing_.mutex_wait_ms = mutex_wait_ms;
        frame_timing_.drain_previous_ms = drain_ms;
        frame_timing_.e2e_ms = std::numeric_limits<double>::quiet_NaN();
        frame_timing_.frame_ready_to_output_ms = std::numeric_limits<double>::quiet_NaN();
        frame_timing_.callback_to_output_ms = std::numeric_limits<double>::quiet_NaN();
        frame_entered_ = entered;
        frame_ready_ = capture.ready;
        neural_frame_succeeded_ = false;
        traditional_ran_ = false;

        try
        {
            if (neural_frame && neural_backend_ == "openvino" && submitOpenVINO(img_msg, callback_start))
            {
                return;  // Accepted or explicitly rejected; result timer never blocks on inference.
            }
            throughput_.add(DetectorThroughput::ACCEPTED);
            auto armors = detectArmors(img_msg);
            completeFrame(img_msg, std::move(armors), callback_start, neural_frame);
        }
        catch (const std::exception & error)
        {
            markFrameFailure(error.what());
            if (!frame_timing_.result_published) { frame_timing_.core_ms = elapsedMs(callback_start); }
            publishFrameTiming(false);
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "Detector cycle failed: %s", error.what());
        }
    }

    void ArmorDetectorNode::completeFrame(
        const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, std::vector<Armor> armors,
        std::chrono::steady_clock::time_point callback_start, bool neural_frame)
    {
        frame_timing_.detected_armors = static_cast<std::uint32_t>(armors.size());
        float pnp_ms = 0.0F;
        std::size_t pnp_count = 0;
        if (pnp_solver_ != nullptr)
        {
            armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
            armors_msg_.armors.clear();
            marker_array_.markers.clear();
            armor_marker_.id = 0;
            text_marker_.id = 0;

            auto_aim_interfaces::msg::Armor armor_msg;
            for (auto& armor : armors)
            {
                cv::Mat rvec, tvec;
                const auto pnp_start = std::chrono::steady_clock::now();
                const bool success = pnp_solver_->solvePnP(armor, rvec, tvec, img_msg->header.stamp);
                pnp_ms += std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - pnp_start).count();
                ++pnp_count;

                if (success)
                {
                    // Fill basic info
                    armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
                    armor_msg.number = armor.number;

                    // Fill pose
                    armor_msg.pose.position.x = tvec.at<double>(0);
                    armor_msg.pose.position.y = tvec.at<double>(1);
                    armor_msg.pose.position.z = tvec.at<double>(2);
                    armor_msg.yaw_raw = armor.yaw_raw;
                    armor_msg.yaw_best = armor.best_yaw;

                    // rvec to quaternion
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvec, rotation_matrix);
                    tf2::Matrix3x3 tf2_rotation_matrix(
                        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                        rotation_matrix.at<double>(2, 2));
                    tf2::Quaternion tf2_q;
                    tf2_rotation_matrix.getRotation(tf2_q);
                    armor_msg.pose.orientation = tf2::toMsg(tf2_q);

                    // Fill the distance to image center
                    armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

                    // Fill markers
                    armor_marker_.id++;
                    armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
                    armor_marker_.pose = armor_msg.pose;
                    text_marker_.id++;
                    text_marker_.pose.position = armor_msg.pose.position;
                    text_marker_.pose.position.y -= 0.1;
                    text_marker_.text = armor.classfication_result;
                    armors_msg_.armors.emplace_back(armor_msg);
                    marker_array_.markers.emplace_back(armor_marker_);
                    marker_array_.markers.emplace_back(text_marker_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "PnP failed!");
                }
            }

            const auto publish_started = SteadyClock::now();
            armors_pub_->publish(armors_msg_);
            const auto output_end = SteadyClock::now();
            const auto output_ros = now();
            frame_timing_.result_publish_ms =
                std::chrono::duration<double, std::milli>(output_end - publish_started).count();
            frame_timing_.output_stamp = output_ros;
            frame_timing_.result_published = true;
            frame_timing_.published_armors = static_cast<std::uint32_t>(armors_msg_.armors.size());
            frame_timing_.core_ms =
                std::chrono::duration<double, std::milli>(output_end - callback_start).count();
            frame_timing_.callback_to_output_ms =
                std::chrono::duration<double, std::milli>(output_end - frame_entered_).count();
            frame_timing_.e2e_ms = (output_ros - img_msg->header.stamp).seconds() * 1000.0;
            if (frame_timing_.embedded_camera)
            {
                frame_timing_.frame_ready_to_output_ms =
                    std::chrono::duration<double, std::milli>(output_end - frame_ready_).count();
            }
            throughput_.add(DetectorThroughput::PUBLISHED);
            const auto marker_started = SteadyClock::now();
            publishMarkers();
            frame_timing_.marker_publish_ms = elapsedMs(marker_started);
        }

        if (!frame_timing_.result_published) { frame_timing_.core_ms = elapsedMs(callback_start); }
        frame_timing_.pnp_ms = pnp_ms;
        frame_timing_.pnp_count = static_cast<std::uint32_t>(pnp_count);
        if (neural_frame && neural_frame_succeeded_ && !traditional_ran_ && debug_)
        {
            const auto debug_started = SteadyClock::now();
            publishNeuralDebugImage(img_msg, armors);
            frame_timing_.debug_ms += elapsedMs(debug_started);
        }
        publishFrameTiming(true);
    }

    void ArmorDetectorNode::publishFrameTiming(bool completed)
    {
        frame_timing_.completed = completed;
        frame_timing_.traditional_fallback = frame_timing_.detector_mode == "neural" && traditional_ran_;
        frame_timing_.cycle_ms = elapsedMs(frame_entered_);
        frame_timing_.finished_stamp = now();
        if (completed) { throughput_.add(DetectorThroughput::COMPLETED); }
        timing_pub_->publish(frame_timing_);
    }

    void ArmorDetectorNode::markFrameFailure(const std::string & error)
    {
        if (frame_timing_.error.empty()) { throughput_.add(DetectorThroughput::FAILED); }
        frame_timing_.error = error;
    }

    void ArmorDetectorNode::setNeuralTiming(const NeuralDetector::Timing & timing)
    {
        frame_timing_.preprocess_ms = timing.preprocess_ms;
        frame_timing_.inference_ms = timing.inference_ms;
        frame_timing_.postprocess_ms = timing.postprocess_ms;
        frame_timing_.neural_total_ms = timing.total_ms;
    }

    void ArmorDetectorNode::publishThroughput()
    {
        const std::lock_guard<std::mutex> lock(processing_mutex_);
        const auto sample = throughput_.sample();
        auto_aim_interfaces::msg::DebugDetectorFps msg;
        msg.header.stamp = now();
        msg.window_seconds = sample.seconds;
        msg.input_fps = sample.fps[DetectorThroughput::INPUT];
        msg.accepted_fps = sample.fps[DetectorThroughput::ACCEPTED];
        msg.completed_fps = sample.fps[DetectorThroughput::COMPLETED];
        msg.published_fps = sample.fps[DetectorThroughput::PUBLISHED];
        msg.rejected_fps = sample.fps[DetectorThroughput::REJECTED];
        msg.failed_fps = sample.fps[DetectorThroughput::FAILED];
        msg.discarded_fps = sample.fps[DetectorThroughput::DISCARDED];
        msg.input_total = sample.totals[DetectorThroughput::INPUT];
        msg.accepted_total = sample.totals[DetectorThroughput::ACCEPTED];
        msg.completed_total = sample.totals[DetectorThroughput::COMPLETED];
        msg.published_total = sample.totals[DetectorThroughput::PUBLISHED];
        msg.rejected_total = sample.totals[DetectorThroughput::REJECTED];
        msg.failed_total = sample.totals[DetectorThroughput::FAILED];
        msg.discarded_total = sample.totals[DetectorThroughput::DISCARDED];
        if (openvino_detector_)
        {
            msg.pending = static_cast<std::uint32_t>(openvino_detector_->pending());
            msg.capacity = static_cast<std::uint32_t>(openvino_detector_->capacity());
        }
        fps_pub_->publish(msg);
        RCLCPP_INFO(get_logger(),
            "Detector FPS: input %.1f | accepted %.1f | completed %.1f | published %.1f | rejected %.1f",
            msg.input_fps, msg.accepted_fps, msg.completed_fps, msg.published_fps, msg.rejected_fps);
    }

    std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.integer_range.resize(1);
        param_desc.integer_range[0].step = 1;
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 255;
        int binary_thres = declare_parameter("binary_thres", 160, param_desc);

        param_desc.description = "0-BLUE, 1-RED";
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 1;
        auto detect_color = declare_parameter("detect_color", RED, param_desc);

        Detector::LightParams l_params = {
            .min_ratio = declare_parameter("light.min_ratio", 0.1),
            .max_ratio = declare_parameter("light.max_ratio", 0.4),
            .max_angle = declare_parameter("light.max_angle", 40.0)
        };

        Detector::ArmorParams a_params = {
            .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
            .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
            .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
            .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
            .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.0),
            .max_angle = declare_parameter("armor.max_angle", 35.0)
        };

        auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

        // Init classifier
        auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
        auto model_path = pkg_path + "/model/mlp.onnx";
        auto label_path = pkg_path + "/model/label.txt";
        double threshold = this->declare_parameter("classifier_threshold", 0.7);
        std::vector<std::string> ignore_classes =
            this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
        detector->classifier =
            std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

        return detector;
    }

    void ArmorDetectorNode::initNeuralParams()
    {
        rcl_interfaces::msg::ParameterDescriptor mode_desc;
        mode_desc.description =
            "检测模式：traditional=传统识别（默认，原有流程），neural=纯神经网络角点输出后直接进入 PnP";
        detector_mode_str_ = declare_parameter("detector_mode", std::string("traditional"), mode_desc);

        rcl_interfaces::msg::ParameterDescriptor backend_desc;
        backend_desc.read_only = true;
        backend_desc.description = "neural backend: onnxruntime / openvino; restart to change";
        neural_backend_ = declare_parameter("neural_backend", std::string("onnxruntime"), backend_desc);
        if (neural_backend_ != "onnxruntime" && neural_backend_ != "openvino")
        {
            throw std::invalid_argument("neural_backend must be onnxruntime or openvino");
        }
        backend_desc.description = "CPU inference thread budget for the whole model; 0 = auto";
        openvino_config_.threads = declare_parameter("openvino.threads", 0, backend_desc);
        backend_desc.description = "CPU parallel inference streams; 0 = THROUGHPUT auto";
        openvino_config_.streams = declare_parameter("openvino.streams", 0, backend_desc);
        backend_desc.description = "Bounded request/result capacity; 0 = recommended count";
        openvino_config_.requests = declare_parameter("openvino.requests", 0, backend_desc);

        rcl_interfaces::msg::ParameterDescriptor model_desc;
        model_desc.description = "神经网络模型路径，留空则用 share/armor_detector/model/shenzhen-0526.onnx";
        neural_model_path_ = declare_parameter("neural_model_path", std::string(""), model_desc);
        if (neural_model_path_.empty())
        {
            neural_model_path_ = ament_index_cpp::get_package_share_directory("armor_detector") +
                                 "/model/shenzhen-0526.onnx";
        }

        rcl_interfaces::msg::ParameterDescriptor conf_desc;
        conf_desc.description = "神经网络置信度阈值";
        conf_desc.floating_point_range.resize(1);
        conf_desc.floating_point_range[0].from_value = 0.0;
        conf_desc.floating_point_range[0].to_value = 1.0;

        neural_params_.conf_threshold =
            static_cast<float>(declare_parameter("neural_conf_threshold", 0.65, conf_desc));
        neural_params_.nms_threshold =
            static_cast<float>(declare_parameter("neural_nms_threshold", 0.45, conf_desc));
        neural_params_.swap_color = declare_parameter("neural_swap_color", false);

        RCLCPP_INFO(
            this->get_logger(), "检测模式: %s（切到神经网络: ros2 param set %s detector_mode neural）",
            detector_mode_str_.c_str(), this->get_name());
        RCLCPP_INFO(this->get_logger(), "神经网络模型: %s", neural_model_path_.c_str());

        setDetectorMode(detector_mode_str_);
    }

    bool ArmorDetectorNode::setDetectorMode(const std::string& mode)
    {
        if (mode != "traditional" && mode != "neural")
        {
            RCLCPP_WARN(
                this->get_logger(), "detector_mode 只支持 traditional / neural，收到 '%s'，忽略",
                mode.c_str());
            return false;
        }
        detector_mode_str_ = mode;
        const bool was_neural = neural_mode_.exchange(mode == "neural");
        if (was_neural != (mode == "neural")) { ++neural_generation_; }
        if (neural_mode_)
        {
            // 允许切模式后重新尝试加载模型
            neural_load_failed_ = false;
        }
        RCLCPP_INFO(
            this->get_logger(), "检测模式已切换: %s", neural_mode_ ? "neural（神经网络）" : "traditional（传统识别）");
        return true;
    }

    void ArmorDetectorNode::updateNeuralParams()
    {
        const auto started = SteadyClock::now();
        // 同步一次可以在线改的参数
        neural_params_.detect_color = get_parameter("detect_color").as_int();
        neural_params_.ignore_classes = get_parameter("ignore_classes").as_string_array();
        neural_params_.conf_threshold =
            static_cast<float>(get_parameter("neural_conf_threshold").as_double());
        neural_params_.nms_threshold =
            static_cast<float>(get_parameter("neural_nms_threshold").as_double());
        neural_params_.swap_color = get_parameter("neural_swap_color").as_bool();
        frame_timing_.parameter_update_ms += elapsedMs(started);
    }

    bool ArmorDetectorNode::ensureOpenVINODetector()
    {
        if (openvino_detector_) { return true; }
        if (neural_load_failed_) { return false; }
        try
        {
            openvino_detector_ = std::make_unique<OpenVINODetector>(neural_model_path_, openvino_config_);
            RCLCPP_INFO(this->get_logger(), "OpenVINO CPU: threads=%d streams=%d requests=%zu",
                openvino_detector_->threads(), openvino_detector_->streams(), openvino_detector_->capacity());
            return true;
        }
        catch (const std::exception & error)
        {
            neural_load_failed_ = true;
            markFrameFailure(error.what());
            RCLCPP_ERROR(this->get_logger(), "OpenVINO init failed: %s; using traditional detector",
                error.what());
            return false;
        }
    }

    bool ArmorDetectorNode::submitOpenVINO(
        const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
        std::chrono::steady_clock::time_point callback_start)
    {
        const auto model_started = SteadyClock::now();
        const bool model_ready = ensureOpenVINODetector();
        frame_timing_.model_init_ms += elapsedMs(model_started);
        if (!model_ready)
        {
            if (frame_timing_.error.empty())
            {
                markFrameFailure("OpenVINO backend unavailable; using traditional detector");
            }
            return false;
        }
        bool accepted = false;
        try
        {
            const auto bridge_start = std::chrono::steady_clock::now();
            const auto rgb = cv_bridge::toCvShare(img_msg, "rgb8")->image;
            frame_timing_.cv_bridge_ms += elapsedMs(bridge_start);
            updateNeuralParams();
            const auto id = frame_timing_.frame_id;
            if (!openvino_detector_->submit(rgb, id, neural_params_))
            {
                ++openvino_rejected_;
                throughput_.add(DetectorThroughput::REJECTED);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "OpenVINO full: rejected=%llu pending=%zu/%zu (frame not processed)",
                    static_cast<unsigned long long>(openvino_rejected_),
                    openvino_detector_->pending(), openvino_detector_->capacity());
                return true;
            }
            accepted = true;
            throughput_.add(DetectorThroughput::ACCEPTED);
            auto inserted = pending_neural_frames_.emplace(id, PendingNeuralFrame{
                img_msg, frame_timing_, callback_start, frame_entered_, frame_ready_, neural_generation_.load()});
            if (is_record_)
            {
                const auto record_started = SteadyClock::now();
                try
                {
                    cv::Mat bgr;
                    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
                    video_writer_.write(bgr);
                }
                catch (const std::exception & error) { markFrameFailure(error.what()); }
                frame_timing_.record_ms += elapsedMs(record_started);
                inserted.first->second.timing = frame_timing_;
            }
            return true;
        }
        catch (const std::exception & error)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "OpenVINO frame/recording error: %s", error.what());
            markFrameFailure(error.what());
            // Once accepted, never publish a second synchronous result for the same frame.
            return accepted;
        }
    }

    void ArmorDetectorNode::drainOpenVINOResults()
    {
        if (!openvino_detector_) { return; }
        NeuralFrameResult result;
        while (openvino_detector_->tryReceive(result))
        {
            auto it = pending_neural_frames_.find(result.frame_id);
            if (it == pending_neural_frames_.end())
            {
                throughput_.add(DetectorThroughput::DISCARDED);
                continue;
            }
            const auto pending = std::move(it->second);
            pending_neural_frames_.erase(it);
            if (!neural_mode_ || pending.generation != neural_generation_.load())
            {
                throughput_.add(DetectorThroughput::DISCARDED);
                continue;
            }
            frame_timing_ = pending.timing;
            frame_entered_ = pending.entered;
            frame_ready_ = pending.ready;
            last_openvino_timing_ = result.timing;
            setNeuralTiming(result.timing);
            frame_timing_.neural_latency_ms = result.latency_ms;
            frame_timing_.result_wait_ms = result.result_wait_ms;
            traditional_ran_ = false;
            neural_frame_succeeded_ = result.error.empty();
            try
            {
                if (!result.error.empty())
                {
                    markFrameFailure(result.error);
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "OpenVINO frame failed: %s; using traditional detector", result.error.c_str());
                    result.armors = detectArmors(pending.image);
                }
                completeFrame(pending.image, std::move(result.armors), pending.started, true);
            }
            catch (const std::exception & error)
            {
                markFrameFailure(error.what());
                if (!frame_timing_.result_published) { frame_timing_.core_ms = elapsedMs(pending.started); }
                publishFrameTiming(false);
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "OpenVINO result cycle failed: %s", error.what());
            }
        }
    }

    bool ArmorDetectorNode::ensureNeuralDetector()
    {
        if (neural_detector_)
        {
            return true;
        }
        if (neural_load_failed_)
        {
            return false;
        }
        if (!NeuralDetector::available())
        {
            neural_load_failed_ = true;
            RCLCPP_ERROR(
                this->get_logger(), "编译时没有链接 onnxruntime，神经网络模式不可用，继续用传统识别");
            return false;
        }

        try
        {
            neural_detector_ = std::make_unique<NeuralDetector>(neural_model_path_, neural_params_);
            RCLCPP_INFO(this->get_logger(), "神经网络模型加载完成，进入神经网络模式");
            return true;
        }
        catch (const std::exception& e)
        {
            neural_load_failed_ = true;
            markFrameFailure(e.what());
            RCLCPP_ERROR(
                this->get_logger(), "神经网络模型加载失败：%s（本帧起自动退回传统识别）", e.what());
            return false;
        }
    }

    std::vector<Armor> ArmorDetectorNode::runTraditional(const cv::Mat & img)
    {
        const auto params_started = SteadyClock::now();
        detector_->binary_thres = get_parameter("binary_thres").as_int();
        detector_->detect_color = get_parameter("detect_color").as_int();
        detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();
        frame_timing_.parameter_update_ms += elapsedMs(params_started);
        traditional_ran_ = true;
        auto armors = detector_->detect(img);
        const auto & timing = detector_->lastTiming();
        frame_timing_.traditional_preprocess_ms = timing.preprocess_ms;
        frame_timing_.traditional_lights_ms = timing.lights_ms;
        frame_timing_.traditional_match_ms = timing.match_ms;
        frame_timing_.traditional_extract_ms = timing.extract_ms;
        frame_timing_.traditional_classify_ms = timing.classify_ms;
        frame_timing_.traditional_total_ms = timing.total_ms;
        return armors;
    }

    std::vector<Armor> ArmorDetectorNode::detectArmorsByNeural(const cv::Mat& img, bool& traditional_ran)
    {
        // 这里只保留“模型不可用/运行异常”的安全退回；NN 正常但零检出时不会跑传统视觉。
        const auto run_traditional_fallback = [this, &img, &traditional_ran]()
        {
            traditional_ran = true;
            return runTraditional(img);
        };

        if (neural_backend_ == "openvino") { return run_traditional_fallback(); }
        const auto model_started = SteadyClock::now();
        const bool model_ready = ensureNeuralDetector();
        frame_timing_.model_init_ms += elapsedMs(model_started);
        if (!model_ready)
        {
            if (frame_timing_.error.empty())
            {
                markFrameFailure("ONNX Runtime backend unavailable; using traditional detector");
            }
            return run_traditional_fallback();
        }

        updateNeuralParams();
        neural_detector_->setParams(neural_params_);

        std::vector<Armor> armors;
        neural_frame_succeeded_ = false;
        const auto neural_started = SteadyClock::now();
        try
        {
            // 纯神经网络检测：网络角点直接送 PnP，不再执行传统角点精修。
            armors = neural_detector_->detect(img);
            neural_frame_succeeded_ = true;
        }
        catch (const std::exception& e)
        {
            frame_timing_.neural_latency_ms = elapsedMs(neural_started);
            markFrameFailure(e.what());
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000, "神经网络推理异常：%s", e.what());
            return run_traditional_fallback();
        }

        const auto & timing = neural_detector_->lastTiming();
        setNeuralTiming(timing);
        frame_timing_.neural_latency_ms = elapsedMs(neural_started);
        RCLCPP_DEBUG(
            this->get_logger(),
            "神经网络: %zu 个, pre %.2fms | infer %.2fms | post %.2fms | NN total %.2fms",
            armors.size(), timing.preprocess_ms, timing.inference_ms, timing.postprocess_ms,
            timing.total_ms);

        return armors;
    }

    std::vector<Armor> ArmorDetectorNode::detectArmors(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
    {
        const bool use_neural = neural_mode_.load();

        // ROS Image -> cv::Mat toCvShare 不复制像素；单独计时方便排查编码转换/传输问题。
        const auto bridge_start = std::chrono::steady_clock::now();
        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
        frame_timing_.cv_bridge_ms += elapsedMs(bridge_start);

        // 录制视频（会占用 callback 时间，正式性能测试建议关闭 is_record）。
        if (is_record_)
        {
            const auto record_started = SteadyClock::now();
            cv::Mat save_img;
            cv::cvtColor(img, save_img, cv::COLOR_RGB2BGR);
            video_writer_.write(save_img);
            frame_timing_.record_ms += elapsedMs(record_started);
        }

        traditional_ran_ = false;
        std::vector<Armor> armors;
        if (use_neural)
        {
            // 神经网络只需要阵营颜色；不再读取传统二值化/MLP 的运行参数。
            detector_->detect_color = get_parameter("detect_color").as_int();
            armors = detectArmorsByNeural(img, traditional_ran_);
        }
        else
        {
            // 传统视觉流程保持原样。
            armors = runTraditional(img);
        }

        // 传统模式（以及 NN 故障安全退回传统）保持原来的 debug 发布流程。
        // 纯 NN 模式的 result_img 延迟到 PnP 完成后发布，这样才能显示完整 PnP/E2E 时间。
        if (debug_ && (!use_neural || traditional_ran_))
        {
            const auto debug_started = SteadyClock::now();
            const auto final_time = this->now();
            const auto latency = (final_time - img_msg->header.stamp).seconds() * 1000.0;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

            binary_img_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

            std::sort(
                detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
                [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
            std::sort(
                detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
                [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });
            lights_data_pub_->publish(detector_->debug_lights);
            armors_data_pub_->publish(detector_->debug_armors);

            if (!armors.empty() && !armors.front().number_img.empty())
            {
                auto all_num_img = detector_->getAllNumbersImage();
                number_img_pub_.publish(
                    *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
            }

            detector_->drawResults(img);
            cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);

            std::stringstream latency_ss;
            latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
            cv::putText(
                img, latency_ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2);
            result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
            frame_timing_.debug_ms += elapsedMs(debug_started);
        }

        return armors;
    }

    void ArmorDetectorNode::publishNeuralDebugImage(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const std::vector<Armor>& armors)
    {
        if (!neural_detector_ && !openvino_detector_)
        {
            return;
        }

        // Debug 绘制使用独立副本，避免修改订阅到的共享图像数据。
        auto debug_img = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
        drawNeuralResults(debug_img, armors);
        cv::circle(debug_img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);

        const auto & nn = neural_backend_ == "openvino" ? last_openvino_timing_ :
            neural_detector_->lastTiming();
        const auto put_line = [&debug_img](const std::string& text, int y, const cv::Scalar& color)
        {
            cv::putText(
                debug_img, text, cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                color, 2, cv::LINE_AA);
        };

        std::ostringstream line1;
        line1 << std::fixed << std::setprecision(2)
              << "E2E " << frame_timing_.e2e_ms << " ms  |  input age "
              << frame_timing_.input_age_ms << " ms  |  core "
              << frame_timing_.core_ms << " ms";
        put_line(line1.str(), 28, cv::Scalar(0, 255, 0));

        std::ostringstream line2;
        line2 << std::fixed << std::setprecision(2)
              << "NN total " << nn.total_ms << " ms  |  pre " << nn.preprocess_ms
              << "  infer " << nn.inference_ms << "  post " << nn.postprocess_ms;
        put_line(line2.str(), 54, cv::Scalar(0, 255, 255));

        std::ostringstream line3;
        line3 << std::fixed << std::setprecision(2)
              << "cv_bridge " << frame_timing_.cv_bridge_ms << " ms  |  PnP "
              << frame_timing_.pnp_ms << " ms (" << frame_timing_.pnp_count
              << " armors)";
        put_line(line3.str(), 80, cv::Scalar(255, 255, 0));

        result_img_pub_.publish(
            cv_bridge::CvImage(img_msg->header, "rgb8", debug_img).toImageMsg());
    }

    void ArmorDetectorNode::createDebugPublishers()
    {
        lights_data_pub_ =
            this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
        armors_data_pub_ =
            this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

        binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
        number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
        result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
    }

    void ArmorDetectorNode::destroyDebugPublishers()
    {
        lights_data_pub_.reset();
        armors_data_pub_.reset();

        binary_img_pub_.shutdown();
        number_img_pub_.shutdown();
        result_img_pub_.shutdown();
    }

    void ArmorDetectorNode::publishMarkers()
    {
        using Marker = visualization_msgs::msg::Marker;
        armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
        marker_array_.markers.emplace_back(armor_marker_);
        marker_pub_->publish(marker_array_);
    }
} // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
