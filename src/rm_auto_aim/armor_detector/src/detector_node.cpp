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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
    ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
        : Node("armor_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

        // Detector
        detector_ = initDetector();

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
                debug_ = p.as_bool();
                debug_ ? createDebugPublishers() : destroyDebugPublishers();
            });

        //tf
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 图像来源：默认还是订阅 /image_raw（视频回放、分进程都能用）
        // use_embedded_camera:=true 时改为本节点直接调相机 SDK，跳过 ROS 话题
        use_embedded_camera_ = this->declare_parameter("use_embedded_camera", false);
        if (use_embedded_camera_)
        {
            // 没有 mv_camera 发 /camera_info，内参由 initEmbeddedCamera 自己加载
            initEmbeddedCamera();
        }
        else
        {
            cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_info", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
                {
                    cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                    pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d, tf_buffer_);
                    cam_info_sub_.reset();
                });

            std::string transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                this, "/image_raw", std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1),
                transport_, rmw_qos_profile_sensor_data));
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
    }

    ArmorDetectorNode::~ArmorDetectorNode()
    {
#ifdef ARMOR_DETECTOR_WITH_CAMERA
        // 先停采集线程，再释放相机句柄
        camera_running_ = false;
        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }
        if (h_camera_ >= 0)
        {
            CameraUnInit(h_camera_);
            h_camera_ = -1;
        }
#endif
    }

    void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        if (debug_)
        {
            static int fps = 0;
            static auto start_time = this->now();
            if (this->now() - start_time >= rclcpp::Duration::from_seconds(1.0))
            {
                RCLCPP_INFO(rclcpp::get_logger("armor_detector"), "ArmorDetector FPS: %d", fps);
                fps = 0;
                start_time = this->now();
            }
            fps++;
        }
        //装甲板识别
        auto armors = detectArmors(img_msg);

        if (pnp_solver_ != nullptr)
        {
            armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
            //todo:要把时间改回图像时间，使用系统时间仅供调试
            // armors_msg_.header.stamp = armor_marker_.header.stamp = text_marker_.header.stamp = this->get_clock()->now();
            armors_msg_.armors.clear();
            marker_array_.markers.clear();
            armor_marker_.id = 0;
            text_marker_.id = 0;

            auto_aim_interfaces::msg::Armor armor_msg;
            for (auto& armor : armors)
            {
                cv::Mat rvec, tvec;
                //todo:要把时间改回图像时间，使用系统时间仅供调试，记得改回来！
                bool success = pnp_solver_->solvePnP(armor, rvec, tvec, img_msg->header.stamp);
                // bool success = pnp_solver_->solvePnP(armor, rvec, tvec, this->get_clock()->now());
                if (success)
                {
                    // Fill basic info
                    armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
                    armor_msg.number = armor.number;

                    // Fill pose
                    armor_msg.pose.position.x = tvec.at<double>(0);
                    armor_msg.pose.position.y = tvec.at<double>(1);
                    armor_msg.pose.position.z = tvec.at<double>(2);
                    //debug
                    armor_msg.yaw_raw = armor.yaw_raw;
                    armor_msg.yaw_best = armor.best_yaw;
                    // rvec to 3x3 rotation matrix
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvec, rotation_matrix);
                    // rotation matrix to quaternion
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

                    // Fill the markers
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

            // Publishing detected armors
            armors_pub_->publish(armors_msg_);

            // Publishing marker
            publishMarkers();
        }
    }

    // ===== 内嵌相机模式：初始化 MindVision SDK =====
    void ArmorDetectorNode::initEmbeddedCamera()
    {
#ifdef ARMOR_DETECTOR_WITH_CAMERA
        CameraSdkInit(1);

        int counts = 1;
        tSdkCameraDevInfo dev_info;
        if (CameraEnumerateDevice(&dev_info, &counts) != CAMERA_STATUS_SUCCESS || counts == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "use_embedded_camera:=true 但没找到相机，收不到图");
            return;
        }
        if (CameraInit(&dev_info, -1, -1, &h_camera_) != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "相机初始化失败");
            h_camera_ = -1;
            return;
        }

        CameraGetCapability(h_camera_, &capability_);
        CameraSetAeState(h_camera_, false);

        // 参数名和 mv_camera_node 保持一致，可以沿用同一份 params.yaml
        double exposure_line_time = 0.0;
        CameraGetExposureLineTime(h_camera_, &exposure_line_time);
        (void)exposure_line_time;  // -Wall -Werror 下避免 unused 告警

        CameraSetExposureTime(h_camera_, this->declare_parameter("exposure_time", 5000));

        int analog_gain = 0;
        CameraGetAnalogGain(h_camera_, &analog_gain);
        CameraSetAnalogGain(h_camera_, this->declare_parameter("analog_gain", analog_gain));

        int r_gain = 1, g_gain = 1, b_gain = 1;
        CameraGetGain(h_camera_, &r_gain, &g_gain, &b_gain);
        r_gain = this->declare_parameter("rgb_gain.r", r_gain);
        g_gain = this->declare_parameter("rgb_gain.g", g_gain);
        b_gain = this->declare_parameter("rgb_gain.b", b_gain);
        CameraSetGain(h_camera_, r_gain, g_gain, b_gain);

        flip_image_ = this->declare_parameter("flip_image", false);

        CameraPlay(h_camera_);
        CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8);

        // 内嵌模式没有 mv_camera 发 /camera_info，自己加载内参并转发给下游
        auto info_url = this->declare_parameter(
            "camera_info_url", std::string("package://bringup/config/camera_info.yaml"));
        camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "mv_camera");
        if (camera_info_manager_->validateURL(info_url) && camera_info_manager_->loadCameraInfo(info_url))
        {
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_manager_->getCameraInfo());
            cam_info_->header.frame_id = "camera_optical_frame";
            cam_center_ = cv::Point2f(cam_info_->k[2], cam_info_->k[5]);
            pnp_solver_ = std::make_unique<PnPSolver>(cam_info_->k, cam_info_->d, tf_buffer_);
            cam_info_pub_ =
                this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", rclcpp::SensorDataQoS());
            RCLCPP_INFO(
                this->get_logger(), "内参: %s (fx=%.1f fy=%.1f cx=%.1f cy=%.1f)", info_url.c_str(),
                cam_info_->k[0], cam_info_->k[4], cam_info_->k[2], cam_info_->k[5]);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "内参加载失败: %s，PnP 将不可用", info_url.c_str());
        }

        camera_running_ = true;
        capture_thread_ = std::thread([this]() { captureLoop(); });
        RCLCPP_INFO(
            this->get_logger(), "内嵌相机已启动：SDK 出图直接调 imageCallback，跳过 /image_raw");
#else
        RCLCPP_ERROR(this->get_logger(), "编译时未链接相机 SDK，use_embedded_camera 不可用");
#endif
    }

    // ===== 内嵌相机模式：采集线程 =====
    // 只做"取图 -> 拼成 Image 消息 -> 调原回调"，识别逻辑一行不改
    void ArmorDetectorNode::captureLoop()
    {
#ifdef ARMOR_DETECTOR_WITH_CAMERA
        // 复用同一个消息对象，避免每帧重新分配
        auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
        img_msg->encoding = "rgb8";
        img_msg->is_bigendian = 0;
        img_msg->header.frame_id = "camera_optical_frame";

        while (rclcpp::ok() && camera_running_.load())
        {
            int status = CameraGetImageBuffer(h_camera_, &frame_info_, &pby_buffer_, 1000);
            if (status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "CameraGetImageBuffer failed, status = %d", status);
                continue;
            }

            const int w = frame_info_.iWidth;
            const int h = frame_info_.iHeight;
            img_msg->width = w;
            img_msg->height = h;
            img_msg->step = w * 3;
            img_msg->data.resize(static_cast<size_t>(w) * h * 3);
            img_msg->header.stamp = this->now();

            CameraImageProcess(h_camera_, pby_buffer_, img_msg->data.data(), &frame_info_);
            if (flip_image_)
            {
                CameraFlipFrameBuffer(img_msg->data.data(), &frame_info_, 3);
            }

            // 先还 buffer，检测期间不再占用内核缓冲，否则相机侧会被阻塞
            CameraReleaseImageBuffer(h_camera_, pby_buffer_);

            // 内参跟帧一起发，下游（tracker_overlay 等）照常能用
            if (cam_info_ && cam_info_pub_)
            {
                cam_info_->header.stamp = img_msg->header.stamp;
                cam_info_pub_->publish(*cam_info_);
            }

            // 直接复用原来的回调：检测 / PnP / 发布全部走原逻辑
            imageCallback(img_msg);
        }
#endif
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

    std::vector<Armor> ArmorDetectorNode::detectArmors(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
    {
        // Convert ROS img to cv::Mat
        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

        // 录制视频
        if (is_record_)
        {
            // img转换为BGR
            cv::Mat save_img;
            cv::cvtColor(img, save_img, cv::COLOR_RGB2BGR);
            video_writer_.write(save_img);
        }

        // Update params
        detector_->binary_thres = get_parameter("binary_thres").as_int();
        detector_->detect_color = get_parameter("detect_color").as_int();
        detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

        auto armors = detector_->detect(img);

        auto final_time = this->now();
        auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

        // Publish debug info
        if (debug_)
        {
            binary_img_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

            // Sort lights and armors data by x coordinate
            std::sort(
                detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
                [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
            std::sort(
                detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
                [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });

            lights_data_pub_->publish(detector_->debug_lights);
            armors_data_pub_->publish(detector_->debug_armors);

            if (!armors.empty())
            {
                auto all_num_img = detector_->getAllNumbersImage();
                number_img_pub_.publish(
                    *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
            }

            detector_->drawResults(img);
            // Draw camera center
            cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
            // Draw latency
            std::stringstream latency_ss;
            latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
            auto latency_s = latency_ss.str();
            cv::putText(
                img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
        }

        return armors;
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
