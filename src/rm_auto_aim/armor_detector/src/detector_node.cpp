// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include "armor_detector/detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <chrono>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
    ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
        : Node("armor_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

        // Detector
        detector_ = initDetector();

        // Neural detector parameters. The model itself is still lazy-loaded by the worker thread.
        initNeuralParams();

        // Armors Publisher
        armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
            "/detector/armors", rclcpp::SensorDataQoS());

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

        // Debug publishers are created once. Runtime debug switching only changes an atomic flag,
        // so the parameter callback never races with the processing worker over publisher lifetime.
        debug_.store(this->declare_parameter("debug", false));
        performance_log_.store(this->declare_parameter("performance_log", true));
        createDebugPublishers();

        parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        debug_cb_handle_ = parameter_event_handler_->add_parameter_callback(
            "debug", [this](const rclcpp::Parameter& p) { debug_.store(p.as_bool()); });
        perf_log_cb_handle_ = parameter_event_handler_->add_parameter_callback(
            "performance_log",
            [this](const rclcpp::Parameter& p) { performance_log_.store(p.as_bool()); });
        mode_cb_handle_ = parameter_event_handler_->add_parameter_callback(
            "detector_mode",
            [this](const rclcpp::Parameter& p) { setDetectorMode(p.as_string()); });

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Camera info is received once. The worker takes a shared_ptr snapshot of PnPSolver each frame.
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
            {
                auto solver = std::make_shared<PnPSolver>(camera_info->k, camera_info->d, tf_buffer_);
                const cv::Point2f center(camera_info->k[2], camera_info->k[5]);

                {
                    std::lock_guard<std::mutex> lock(camera_state_mutex_);
                    cam_center_ = center;
                    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                    pnp_solver_ = std::move(solver);
                }

                cam_info_sub_.reset();
            });

        // Image subscription: callback only writes the latest-frame mailbox.
        const std::string transport =
            this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        auto qos_image_sub = rmw_qos_profile_sensor_data;
        qos_image_sub.depth = 1;
        img_sub_ = std::make_shared<image_transport::Subscriber>(
            image_transport::create_subscription(
                this, "/image_raw",
                std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1), transport,
                qos_image_sub));

        // Video recording
        is_record_ = this->declare_parameter("is_record", false);
        if (is_record_)
        {
            const std::string save_video_path = this->declare_parameter("save_video_path", "armor.avi");
            const int save_video_fps = this->declare_parameter("save_video_fps", 30);
            const int save_video_width = this->declare_parameter("save_video_width", 640);
            const int save_video_height = this->declare_parameter("save_video_height", 480);
            video_writer_.open(
                save_video_path, cv::VideoWriter::fourcc('P', 'I', 'M', '1'), save_video_fps,
                cv::Size(save_video_width, save_video_height), true);
        }

        fps_window_start_ = std::chrono::steady_clock::now();
        processing_thread_ = std::thread(&ArmorDetectorNode::processingLoop, this);

        RCLCPP_INFO(
            this->get_logger(),
            "Latest-frame worker enabled: subscription callback only keeps newest frame (QoS depth=1)");
    }

    ArmorDetectorNode::~ArmorDetectorNode()
    {
        worker_running_.store(false);
        frame_cv_.notify_all();
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }

        if (video_writer_.isOpened())
        {
            video_writer_.release();
        }
    }

    void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        FramePacket frame;
        frame.image = img_msg;
        frame.received_steady = std::chrono::steady_clock::now();
        frame.rx_age_ms = static_cast<float>((this->now() - img_msg->header.stamp).seconds() * 1000.0);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (latest_frame_available_)
            {
                overwritten_frames_.fetch_add(1, std::memory_order_relaxed);
            }
            latest_frame_ = std::move(frame);
            latest_frame_available_ = true;
        }

        received_frames_.fetch_add(1, std::memory_order_relaxed);
        frame_cv_.notify_one();
    }

    void ArmorDetectorNode::processingLoop()
    {
        while (worker_running_.load())
        {
            FramePacket frame;
            {
                std::unique_lock<std::mutex> lock(frame_mutex_);
                frame_cv_.wait(lock, [this]()
                {
                    return latest_frame_available_ || !worker_running_.load();
                });

                if (!worker_running_.load())
                {
                    break;
                }

                frame = std::move(latest_frame_);
                latest_frame_ = {};
                latest_frame_available_ = false;
            }

            if (frame.image)
            {
                try
                {
                    processFrame(frame);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "Frame processing exception: %s", e.what());
                }
            }
        }
    }

    void ArmorDetectorNode::processFrame(const FramePacket& frame)
    {
        FrameTiming timing;
        timing.neural_requested = neural_mode_.load();
        timing.rx_age_ms = frame.rx_age_ms;

        const auto work_start = std::chrono::steady_clock::now();
        timing.mailbox_wait_ms =
            std::chrono::duration<float, std::milli>(work_start - frame.received_steady).count();
        timing.work_age_ms =
            static_cast<float>((this->now() - frame.image->header.stamp).seconds() * 1000.0);

        // Snapshot camera state. The mutex is held only while copying a shared_ptr and one Point2f.
        std::shared_ptr<PnPSolver> pnp_solver;
        cv::Point2f cam_center;
        {
            std::lock_guard<std::mutex> lock(camera_state_mutex_);
            pnp_solver = pnp_solver_;
            cam_center = cam_center_;
        }

        // ROS Image -> cv::Mat. toCvShare is zero-copy when encoding already matches rgb8.
        const auto bridge_start = std::chrono::steady_clock::now();
        const auto cv_ptr = cv_bridge::toCvShare(frame.image, "rgb8");
        const cv::Mat img = cv_ptr->image;
        timing.cv_bridge_ms =
            std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - bridge_start).count();

        if (is_record_)
        {
            const auto record_start = std::chrono::steady_clock::now();
            cv::Mat save_img;
            cv::cvtColor(img, save_img, cv::COLOR_RGB2BGR);
            video_writer_.write(save_img);
            timing.record_ms =
                std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - record_start).count();
        }

        const auto detect_start = std::chrono::steady_clock::now();
        auto armors = detectArmors(img, timing.neural_requested, timing);
        timing.detect_ms =
            std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - detect_start).count();

        solvePnPAndPublish(armors, frame.image, timing, pnp_solver);

        timing.core_ms =
            std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - work_start).count();
        timing.e2e_ms =
            static_cast<float>((this->now() - frame.image->header.stamp).seconds() * 1000.0);

        ++processed_frames_;
        ++fps_window_frames_;
        const auto fps_now = std::chrono::steady_clock::now();
        const float fps_elapsed =
            std::chrono::duration<float>(fps_now - fps_window_start_).count();
        if (fps_elapsed >= 1.0F)
        {
            processing_fps_ = static_cast<float>(fps_window_frames_) / fps_elapsed;
            fps_window_frames_ = 0;
            fps_window_start_ = fps_now;
        }

        logPerformance(timing);

        // Debug work intentionally happens after core/E2E timing is captured.
        if (debug_.load())
        {
            if (timing.neural_requested && timing.neural_succeeded && !timing.traditional_fallback)
            {
                publishNeuralDebugImage(frame.image, armors, timing, cam_center);
            }
            else
            {
                publishTraditionalDebugImage(frame.image, armors, timing, cam_center);
            }
        }
    }

    std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.integer_range.resize(1);
        param_desc.integer_range[0].step = 1;
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 255;
        const int binary_thres = declare_parameter("binary_thres", 160, param_desc);

        param_desc.description = "0-BLUE, 1-RED";
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 1;
        const auto detect_color = declare_parameter("detect_color", RED, param_desc);

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
        const auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
        const auto model_path = pkg_path + "/model/mlp.onnx";
        const auto label_path = pkg_path + "/model/label.txt";
        const double threshold = this->declare_parameter("classifier_threshold", 0.7);
        const std::vector<std::string> ignore_classes =
            this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
        detector->classifier =
            std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

        return detector;
    }

    void ArmorDetectorNode::initNeuralParams()
    {
        rcl_interfaces::msg::ParameterDescriptor mode_desc;
        mode_desc.description =
            "检测模式：traditional=传统识别，neural=纯神经网络角点输出后直接进入 PnP";
        detector_mode_str_ = declare_parameter("detector_mode", std::string("traditional"), mode_desc);

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
        neural_mode_.store(mode == "neural");
        if (neural_mode_.load())
        {
            // Allow a new load attempt after switching into neural mode.
            neural_load_failed_.store(false);
        }

        RCLCPP_INFO(
            this->get_logger(), "检测模式已切换: %s",
            neural_mode_.load() ? "neural（神经网络）" : "traditional（传统识别）");
        return true;
    }

    bool ArmorDetectorNode::ensureNeuralDetector()
    {
        if (neural_detector_)
        {
            return true;
        }
        if (neural_load_failed_.load())
        {
            return false;
        }
        if (!NeuralDetector::available())
        {
            neural_load_failed_.store(true);
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
            neural_load_failed_.store(true);
            RCLCPP_ERROR(
                this->get_logger(), "神经网络模型加载失败：%s（自动退回传统识别）", e.what());
            return false;
        }
    }

    std::vector<Armor> ArmorDetectorNode::detectArmors(
        const cv::Mat& img, bool use_neural, FrameTiming& timing)
    {
        if (use_neural)
        {
            return detectArmorsNeural(img, timing);
        }

        return detectArmorsTraditional(img);
    }

    std::vector<Armor> ArmorDetectorNode::detectArmorsTraditional(const cv::Mat& img)
    {
        // Keep the original traditional-vision algorithm and its parameter semantics unchanged.
        detector_->binary_thres = get_parameter("binary_thres").as_int();
        detector_->detect_color = get_parameter("detect_color").as_int();
        detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();
        return detector_->detect(img);
    }

    std::vector<Armor> ArmorDetectorNode::detectArmorsNeural(
        const cv::Mat& img, FrameTiming& timing)
    {
        const auto run_traditional_fallback = [this, &img, &timing]()
        {
            timing.traditional_fallback = true;
            return detectArmorsTraditional(img);
        };

        if (!ensureNeuralDetector())
        {
            return run_traditional_fallback();
        }

        neural_params_.detect_color = get_parameter("detect_color").as_int();
        neural_params_.ignore_classes = get_parameter("ignore_classes").as_string_array();
        neural_params_.conf_threshold =
            static_cast<float>(get_parameter("neural_conf_threshold").as_double());
        neural_params_.nms_threshold =
            static_cast<float>(get_parameter("neural_nms_threshold").as_double());
        neural_params_.swap_color = get_parameter("neural_swap_color").as_bool();
        neural_detector_->setParams(neural_params_);

        try
        {
            // Network corners go directly into PnP; no traditional corner refinement is performed.
            auto armors = neural_detector_->detect(img);
            timing.neural_succeeded = true;
            return armors;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000, "神经网络推理异常：%s", e.what());
            return run_traditional_fallback();
        }
    }

    void ArmorDetectorNode::solvePnPAndPublish(
        std::vector<Armor>& armors,
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        FrameTiming& timing,
        const std::shared_ptr<PnPSolver>& pnp_solver)
    {
        if (!pnp_solver)
        {
            return;
        }

        armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
        armors_msg_.armors.clear();
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;

        for (auto& armor : armors)
        {
            cv::Mat rvec;
            cv::Mat tvec;

            const auto pnp_start = std::chrono::steady_clock::now();
            const bool success = pnp_solver->solvePnP(armor, rvec, tvec, img_msg->header.stamp);
            timing.pnp_ms +=
                std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - pnp_start).count();
            ++timing.pnp_count;

            if (!success)
            {
                RCLCPP_WARN(this->get_logger(), "PnP failed!");
                continue;
            }

            auto_aim_interfaces::msg::Armor armor_msg;
            armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
            armor_msg.number = armor.number;

            armor_msg.pose.position.x = tvec.at<double>(0);
            armor_msg.pose.position.y = tvec.at<double>(1);
            armor_msg.pose.position.z = tvec.at<double>(2);
            armor_msg.yaw_raw = armor.yaw_raw;
            armor_msg.yaw_best = armor.best_yaw;

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

            armor_msg.distance_to_image_center = pnp_solver->calculateDistanceToCenter(armor.center);

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

        const auto publish_start = std::chrono::steady_clock::now();
        armors_pub_->publish(armors_msg_);
        publishMarkers();
        timing.publish_ms =
            std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - publish_start).count();
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

    void ArmorDetectorNode::publishTraditionalDebugImage(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const std::vector<Armor>& armors,
        const FrameTiming& timing,
        const cv::Point2f& cam_center)
    {
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
            const auto all_num_img = detector_->getAllNumbersImage();
            number_img_pub_.publish(
                *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
        }

        auto debug_img = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
        detector_->drawResults(debug_img);
        cv::circle(debug_img, cam_center, 5, cv::Scalar(255, 0, 0), 2);
        drawPerformanceOverlay(debug_img, timing, nullptr);

        result_img_pub_.publish(
            cv_bridge::CvImage(img_msg->header, "rgb8", debug_img).toImageMsg());
    }

    void ArmorDetectorNode::publishNeuralDebugImage(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const std::vector<Armor>& armors,
        const FrameTiming& timing,
        const cv::Point2f& cam_center)
    {
        if (!neural_detector_)
        {
            return;
        }

        auto debug_img = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
        neural_detector_->drawResults(debug_img, armors);
        cv::circle(debug_img, cam_center, 5, cv::Scalar(255, 0, 0), 2);

        const auto& nn_timing = neural_detector_->lastTiming();
        drawPerformanceOverlay(debug_img, timing, &nn_timing);

        result_img_pub_.publish(
            cv_bridge::CvImage(img_msg->header, "rgb8", debug_img).toImageMsg());
    }

    void ArmorDetectorNode::drawPerformanceOverlay(
        cv::Mat& img,
        const FrameTiming& timing,
        const NeuralDetector::Timing* neural_timing) const
    {
        const auto put_line = [&img](const std::string& text, int y, const cv::Scalar& color)
        {
            cv::putText(
                img, text, cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv::LINE_AA);
        };

        std::ostringstream line1;
        line1 << std::fixed << std::setprecision(2)
            << (timing.neural_requested ? (timing.traditional_fallback ? "NN->TRAD" : "NN") : "TRAD")
            << "  E2E " << timing.e2e_ms << " ms"
            << " | RX age " << timing.rx_age_ms
            << " | wait " << timing.mailbox_wait_ms
            << " | work age " << timing.work_age_ms;
        put_line(line1.str(), 24, cv::Scalar(0, 255, 0));

        std::ostringstream line2;
        line2 << std::fixed << std::setprecision(2);
        if (neural_timing != nullptr)
        {
            line2 << "NN " << neural_timing->total_ms << " ms"
                << " | pre " << neural_timing->preprocess_ms
                << " | infer " << neural_timing->inference_ms
                << " | post " << neural_timing->postprocess_ms;
        }
        else
        {
            line2 << "detect " << timing.detect_ms << " ms";
        }
        put_line(line2.str(), 48, cv::Scalar(0, 255, 255));

        std::ostringstream line3;
        line3 << std::fixed << std::setprecision(2)
            << "bridge " << timing.cv_bridge_ms
            << " | PnP " << timing.pnp_ms << "(" << timing.pnp_count << ")"
            << " | pub " << timing.publish_ms
            << " | core " << timing.core_ms
            << " | FPS " << processing_fps_;
        put_line(line3.str(), 72, cv::Scalar(255, 255, 0));

        std::ostringstream line4;
        line4 << "RX " << received_frames_.load(std::memory_order_relaxed)
            << " | processed " << processed_frames_
            << " | overwritten " << overwritten_frames_.load(std::memory_order_relaxed);
        if (is_record_)
        {
            line4 << std::fixed << std::setprecision(2) << " | record " << timing.record_ms << " ms";
        }
        put_line(line4.str(), 96, cv::Scalar(255, 180, 80));
    }

    void ArmorDetectorNode::logPerformance(const FrameTiming& timing)
    {
        if (!performance_log_.load())
        {
            return;
        }

        const auto received =
            static_cast<unsigned long long>(received_frames_.load(std::memory_order_relaxed));
        const auto overwritten =
            static_cast<unsigned long long>(overwritten_frames_.load(std::memory_order_relaxed));
        const auto processed = static_cast<unsigned long long>(processed_frames_);

        if (timing.neural_requested && timing.neural_succeeded && !timing.traditional_fallback &&
            neural_detector_)
        {
            const auto& nn = neural_detector_->lastTiming();
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "NN perf: RXage %.2f | wait %.2f | workAge %.2f | bridge %.2f | pre %.2f | infer %.2f | "
                "post %.2f | NN %.2f | PnP %.2f(%zu) | pub %.2f | core %.2f | E2E %.2f ms | "
                "FPS %.1f | RX %llu processed %llu overwritten %llu",
                timing.rx_age_ms, timing.mailbox_wait_ms, timing.work_age_ms, timing.cv_bridge_ms,
                nn.preprocess_ms, nn.inference_ms, nn.postprocess_ms, nn.total_ms, timing.pnp_ms,
                timing.pnp_count, timing.publish_ms, timing.core_ms, timing.e2e_ms, processing_fps_, received,
                processed, overwritten);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "%s perf: RXage %.2f | wait %.2f | workAge %.2f | bridge %.2f | detect %.2f | "
                "PnP %.2f(%zu) | pub %.2f | core %.2f | E2E %.2f ms | FPS %.1f | "
                "RX %llu processed %llu overwritten %llu",
                timing.traditional_fallback ? "NN->TRAD" : "TRAD", timing.rx_age_ms,
                timing.mailbox_wait_ms, timing.work_age_ms, timing.cv_bridge_ms, timing.detect_ms,
                timing.pnp_ms, timing.pnp_count, timing.publish_ms, timing.core_ms, timing.e2e_ms,
                processing_fps_, received, processed, overwritten);
        }
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

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
