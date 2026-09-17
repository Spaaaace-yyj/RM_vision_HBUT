#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <auto_aim_interfaces/msg/target.hpp>

namespace rm_auto_aim
{
    class TrackerOverlayNode : public rclcpp::Node
    {
    public:
        explicit TrackerOverlayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node("tracker_overlay_node", options),
              tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
              tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
        {
            RCLCPP_INFO(this->get_logger(), "Starting TrackerOverlayNode!");

            debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/detector/result_img");
            target_topic_ = declare_parameter<std::string>("target_topic", "/tracker/target");
            camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info");
            output_topic_ = declare_parameter<std::string>("output_topic", "/debug/tracker_overlay_img");

            // 如果为空，就使用图像 header.frame_id。
            // 如果你的图像 frame 不是 optical frame，可以在 launch 里指定 camera_frame_id。
            camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "");

            small_armor_width_ = declare_parameter<double>("small_armor_width", 0.135);
            large_armor_width_ = declare_parameter<double>("large_armor_width", 0.23);
            armor_height_ = declare_parameter<double>("armor_height", 0.125);

            draw_rect_ = declare_parameter<bool>("draw_rect", true);
            draw_center_ = declare_parameter<bool>("draw_center", true);
            draw_text_ = declare_parameter<bool>("draw_text", true);

            tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.03);
            sync_queue_size_ = declare_parameter<int>("sync_queue_size", 20);

            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&TrackerOverlayNode::cameraInfoCallback, this, std::placeholders::_1));

            overlay_pub_ = image_transport::create_publisher(this, output_topic_);

            rclcpp::SensorDataQoS sensor_qos;

            image_sub_.subscribe(this, debug_image_topic_, sensor_qos.get_rmw_qos_profile());
            target_sub_.subscribe(this, target_topic_, sensor_qos.get_rmw_qos_profile());

            sync_ = std::make_shared<Synchronizer>(
                SyncPolicy(sync_queue_size_),
                image_sub_,
                target_sub_);

            sync_->registerCallback(
                std::bind(
                    &TrackerOverlayNode::syncCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }

    private:
        using ImageMsg = sensor_msgs::msg::Image;
        using TargetMsg = auto_aim_interfaces::msg::Target;

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, TargetMsg>;
        using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

        struct ArmorDrawInfo
        {
            int id = 0;
            Eigen::Vector3d center;
            double yaw = 0.0;
            double width = 0.135;
            double height = 0.125;
        };

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
        {
            camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
            camera_matrix_.at<double>(0, 0) = msg->k[0];
            camera_matrix_.at<double>(0, 1) = msg->k[1];
            camera_matrix_.at<double>(0, 2) = msg->k[2];
            camera_matrix_.at<double>(1, 0) = msg->k[3];
            camera_matrix_.at<double>(1, 1) = msg->k[4];
            camera_matrix_.at<double>(1, 2) = msg->k[5];
            camera_matrix_.at<double>(2, 0) = msg->k[6];
            camera_matrix_.at<double>(2, 1) = msg->k[7];
            camera_matrix_.at<double>(2, 2) = msg->k[8];

            dist_coeffs_ = cv::Mat(msg->d).clone();
            dist_coeffs_ = dist_coeffs_.reshape(1, 1);

            has_camera_info_ = true;

            if (camera_frame_id_.empty())
            {
                camera_frame_id_ = msg->header.frame_id;
            }

            RCLCPP_INFO_ONCE(
                this->get_logger(),
                "Received camera info. camera_frame_id = %s",
                camera_frame_id_.c_str());
        }

        void syncCallback(
            const ImageMsg::ConstSharedPtr image_msg,
            const TargetMsg::ConstSharedPtr target_msg)
        {
            if (!has_camera_info_)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Waiting for camera_info...");
                return;
            }

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
            }
            catch (const cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat image = cv_ptr->image;

            if (!target_msg->tracking)
            {
                drawStatusText(image, "TRACKING: false", cv::Point(10, 60), cv::Scalar(255, 80, 80));
                publishImage(image_msg->header, image);
                return;
            }

            const std::string camera_frame =
                camera_frame_id_.empty() ? image_msg->header.frame_id : camera_frame_id_;

            geometry_msgs::msg::TransformStamped tf_target_to_camera;
            try
            {
                tf_target_to_camera = tf_buffer_->lookupTransform(
                    camera_frame,
                    target_msg->header.frame_id,
                    image_msg->header.stamp,
                    tf2::durationFromSec(tf_timeout_sec_));
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "Could not transform %s -> %s: %s",
                    target_msg->header.frame_id.c_str(),
                    camera_frame.c_str(),
                    ex.what());

                drawStatusText(image, "TF failed", cv::Point(10, 60), cv::Scalar(255, 80, 80));
                publishImage(image_msg->header, image);
                return;
            }

            const auto armor_infos = computeArmorDrawInfos(*target_msg);

            for (const auto& armor : armor_infos)
            {
                drawProjectedArmor(image, armor, tf_target_to_camera);
            }

            drawTargetInfo(image, *target_msg);

            publishImage(image_msg->header, image);
        }

        std::vector<ArmorDrawInfo> computeArmorDrawInfos(const TargetMsg& target_msg) const
        {
            std::vector<ArmorDrawInfo> armors;

            const int armor_num = std::max(1, target_msg.armors_num);

            const double yaw = target_msg.yaw;
            const double r1 = target_msg.radius_1;
            const double r2 = target_msg.radius_2;

            const double xc = target_msg.position.x;
            const double yc = target_msg.position.y;
            const double za = target_msg.position.z;

            const double dz = target_msg.dz;

            const double width =
                target_msg.type == "small" ? small_armor_width_ : large_armor_width_;

            bool is_current_pair = true;

            for (int i = 0; i < armor_num; ++i)
            {
                const double tmp_yaw = yaw + i * 2.0 * M_PI / static_cast<double>(armor_num);

                double r = r1;
                double z = za;

                // 和 tracker_node.cpp 里的 publishMarkers() 保持一致：
                // 4 装甲板时两组半径、高度交替使用。
                if (armor_num == 4)
                {
                    r = is_current_pair ? r1 : r2;
                    z = za + (is_current_pair ? 0.0 : dz);
                    is_current_pair = !is_current_pair;
                }

                ArmorDrawInfo info;
                info.id = i;
                info.center = Eigen::Vector3d(
                    xc - r * std::cos(tmp_yaw),
                    yc - r * std::sin(tmp_yaw),
                    z);
                info.yaw = tmp_yaw;
                info.width = width;
                info.height = armor_height_;

                armors.emplace_back(info);
            }

            return armors;
        }

        std::vector<Eigen::Vector3d> computeArmorCorners(const ArmorDrawInfo& armor) const
        {
            // 坐标约定与 tracker 中装甲板中心计算保持一致：
            // 装甲板中心在 target frame 下。
            // 宽度方向取水平切向方向，竖直方向取 z 轴。
            const Eigen::Vector3d tangent(
                -std::sin(armor.yaw),
                std::cos(armor.yaw),
                0.0);

            const Eigen::Vector3d up(0.0, 0.0, 1.0);

            const Eigen::Vector3d half_w = 0.5 * armor.width * tangent;
            const Eigen::Vector3d half_h = 0.5 * armor.height * up;

            std::vector<Eigen::Vector3d> corners;
            corners.reserve(4);

            corners.emplace_back(armor.center - half_w - half_h);
            corners.emplace_back(armor.center + half_w - half_h);
            corners.emplace_back(armor.center + half_w + half_h);
            corners.emplace_back(armor.center - half_w + half_h);

            return corners;
        }

        bool transformPointToCamera(
            const Eigen::Vector3d& p_target,
            const geometry_msgs::msg::TransformStamped& tf_target_to_camera,
            cv::Point3f& p_camera) const
        {
            geometry_msgs::msg::PointStamped src;
            src.header = tf_target_to_camera.header;
            src.point.x = p_target.x();
            src.point.y = p_target.y();
            src.point.z = p_target.z();

            geometry_msgs::msg::PointStamped dst;
            tf2::doTransform(src, dst, tf_target_to_camera);

            // OpenCV 相机坐标要求 z > 0。
            // 如果你的 camera_frame_id 不是 optical frame，这里会不对。
            if (dst.point.z <= 1e-3)
            {
                return false;
            }

            p_camera = cv::Point3f(
                static_cast<float>(dst.point.x),
                static_cast<float>(dst.point.y),
                static_cast<float>(dst.point.z));

            return true;
        }

        bool projectPoint(
            const cv::Point3f& p_camera,
            cv::Point2f& uv) const
        {
            std::vector<cv::Point3f> object_points{p_camera};
            std::vector<cv::Point2f> image_points;

            const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

            cv::projectPoints(
                object_points,
                rvec,
                tvec,
                camera_matrix_,
                dist_coeffs_,
                image_points);

            if (image_points.empty())
            {
                return false;
            }

            uv = image_points[0];
            return std::isfinite(uv.x) && std::isfinite(uv.y);
        }

        void drawProjectedArmor(
            cv::Mat& image,
            const ArmorDrawInfo& armor,
            const geometry_msgs::msg::TransformStamped& tf_target_to_camera)
        {
            cv::Point3f center_camera;
            cv::Point2f center_uv;

            if (!transformPointToCamera(armor.center, tf_target_to_camera, center_camera))
            {
                return;
            }

            if (!projectPoint(center_camera, center_uv))
            {
                return;
            }

            const cv::Scalar center_color(0, 255, 0);
            const cv::Scalar rect_color(255, 255, 0);
            const cv::Scalar text_color(255, 255, 255);

            if (draw_center_)
            {
                cv::circle(image, center_uv, 4, center_color, -1);
            }

            if (draw_rect_)
            {
                const auto corners_target = computeArmorCorners(armor);

                std::vector<cv::Point> corners_uv;
                corners_uv.reserve(4);

                bool all_projected = true;

                for (const auto& p_target : corners_target)
                {
                    cv::Point3f p_cam;
                    cv::Point2f uv;

                    if (!transformPointToCamera(p_target, tf_target_to_camera, p_cam) || !projectPoint(p_cam, uv))
                    {
                        all_projected = false;
                        break;
                    }

                    corners_uv.emplace_back(
                        static_cast<int>(std::round(uv.x)),
                        static_cast<int>(std::round(uv.y)));
                }

                if (all_projected && corners_uv.size() == 4)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        cv::line(
                            image,
                            corners_uv[i],
                            corners_uv[(i + 1) % 4],
                            rect_color,
                            2);
                    }
                }
            }

            if (draw_text_)
            {
                std::ostringstream ss;
                ss << "armor " << armor.id;

                cv::putText(
                    image,
                    ss.str(),
                    center_uv + cv::Point2f(6.0f, -6.0f),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    text_color,
                    1);
            }
        }

        void drawTargetInfo(cv::Mat& image, const TargetMsg& target_msg) const
        {
            std::ostringstream ss;
            ss << "TRACKING id=" << target_msg.id
                << " yaw=" << std::fixed << std::setprecision(2) << target_msg.yaw
                << " vyaw=" << std::fixed << std::setprecision(2) << target_msg.v_yaw;

            drawStatusText(image, ss.str(), cv::Point(10, 60), cv::Scalar(0, 255, 0));
        }

        void drawStatusText(
            cv::Mat& image,
            const std::string& text,
            const cv::Point& origin,
            const cv::Scalar& color) const
        {
            cv::putText(
                image,
                text,
                origin,
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2);
        }

        void publishImage(
            const std_msgs::msg::Header& header,
            const cv::Mat& image)
        {
            overlay_pub_.publish(
                cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image).toImageMsg());
        }

    private:
        std::string debug_image_topic_;
        std::string target_topic_;
        std::string camera_info_topic_;
        std::string output_topic_;
        std::string camera_frame_id_;

        double small_armor_width_ = 0.135;
        double large_armor_width_ = 0.23;
        double armor_height_ = 0.125;

        bool draw_rect_ = true;
        bool draw_center_ = true;
        bool draw_text_ = true;

        double tf_timeout_sec_ = 0.03;
        int sync_queue_size_ = 20;

        bool has_camera_info_ = false;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        message_filters::Subscriber<ImageMsg> image_sub_;
        message_filters::Subscriber<TargetMsg> target_sub_;
        std::shared_ptr<Synchronizer> sync_;

        image_transport::Publisher overlay_pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
} // namespace rm_auto_aim

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rm_auto_aim::TrackerOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
