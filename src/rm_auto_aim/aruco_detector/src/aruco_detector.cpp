//
// Created by spaaaaace on 2026/2/5.
//
#include "../include/aruco_detector/aruco_detector.h"

ArucoDetector::ArucoDetector() : Node("ArucoDetector")
{
    //sub
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));
    //pub
    armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>("detector/armors", 10);
    marker_arrau_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("debug/target_raw", 10);
    //tf
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    float aruco_size = 0.04;    //m
    aruco_object_.push_back(cv::Point3f(-(aruco_size / 2), aruco_size / 2, 0));
    aruco_object_.push_back(cv::Point3f(aruco_size / 2, aruco_size / 2, 0));
    aruco_object_.push_back(cv::Point3f(aruco_size / 2, -(aruco_size / 2), 0));
    aruco_object_.push_back(cv::Point3f(-(aruco_size / 2), -(aruco_size / 2), 0));

    RCLCPP_INFO(this->get_logger(), "Aruco Detector Init!");
}

void ArucoDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat frame;
    //异常处理
    if (msg->width == 0 || msg->height == 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid image size");
        return;
    }
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }catch (cv_bridge::Exception &error)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", error.what());
    }

    if (cv_ptr->image.empty())
    {
        RCLCPP_WARN(this->get_logger(), "cv image empty");
    }
    image_raw_ = cv_ptr->image.clone();
    frame = image_raw_.clone();

    auto_aim_interfaces::msg::Armors armors;
    armors.header.stamp = this->get_clock()->now();
    armors.header.frame_id = "camera_link";

    cv::Point2f image_center(frame.cols / 2, frame.rows / 2);

    std::vector<int> markerIds; //Aruco码ID
    std::vector<std::vector<cv::Point2f> > markerCorners;   //识别出的角点
    std::vector<std::vector<cv::Point2f> > rejectedCandidates;  //不符合要求的目标

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
                        cv::aruco::DetectorParameters::create();

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    //检测Aruco
    cv::aruco::detectMarkers(
        frame,
        dictionary,
        markerCorners,
        markerIds,
        detectorParams,
        rejectedCandidates
    );

    std::vector<Aruco> arucos;

    for (size_t i = 0; i < markerIds.size(); i++)
    {
        auto_aim_interfaces::msg::Armor armor;
        Aruco aruco(markerIds[i], markerCorners[i], aruco_object_);
        if (!aruco.solveArucoPnP(camera_matrix_, dist_coeffs_))
        {
            RCLCPP_WARN(this->get_logger(), "Can't solve PNP! Skip this Aruco");
            continue;
        }
        cv::Point2f aruco_center = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) / 4;
        float distance_to_image_center = cv::norm(aruco_center - image_center);
        float distance_to_camera_center = cv::norm(aruco.world_location_);
        armor.number = std::to_string(aruco.id_);
        armor.type = "small";
        armor.distance_to_image_center = distance_to_image_center;
        armor.distance_to_camera_center = distance_to_camera_center;
        armor.pose = aruco.aruco_pos_cv_;

        armors.armors.push_back(armor);
        arucos.push_back(aruco);
    }
    armors_pub_->publish(armors);


    if (debug_msg)
    {
        visualization_msgs::msg::MarkerArray markers;
        for (size_t i = 0; i < arucos.size(); i++)
        {
            visualization_msgs::msg::Marker marker;

            marker.header.stamp = now();
            marker.header.frame_id = "camera_link";

            marker.ns = "aruco" + std::to_string(arucos[i].id_);
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = arucos[i].aruco_pos_ros_;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.01;

            marker.color.r = 0.2;
            marker.color.g = 0.9;
            marker.color.b = 0.2;
            marker.color.a = 0.4;
            marker.lifetime = rclcpp::Duration::from_seconds(0.1);

            markers.markers.push_back(marker);

            cv::putText(frame, "ID:" + std::to_string(arucos[i].id_), arucos[i].image_point_[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
            cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, arucos[i].rvec_cv, arucos[i].tvec_cv, 0.02, 2);
        }
        cv::imshow("raw_image", frame);
        cv::waitKey(1);
        marker_arrau_pub_->publish(markers);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
