#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        pub_ = image_transport::create_publisher(this, "camera/image_raw");

        timer_send_fps_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            double fps = cap_.get(cv::CAP_PROP_FPS);
            RCLCPP_INFO(this->get_logger(), "camera fps: %.2f FPS", fps);
        });
        
        if (open_specific_camera())
        {
            setup_timer();
        }
    }

private:
    bool open_specific_camera()
    {
        const std::string target_card_type = "OsmoAction4: OsmoAction4";
        for (int i = 0; i < 10; ++i)
        {
            std::string device_path = "/dev/video" + std::to_string(i);
            int fd = open(device_path.c_str(), O_RDONLY);
            if (fd != -1)
            {
                struct v4l2_capability cap;
                if (ioctl(fd, VIDIOC_QUERYCAP, &cap) != -1)
                {
                    std::string card_type(reinterpret_cast<const char*>(cap.card));
                    if (card_type == target_card_type)
                    {
                        RCLCPP_INFO(this->get_logger(), "Find OsmoAction4, device ID: %d", i);
                        cap_.open(i, cv::CAP_V4L2);
                        if (cap_.isOpened())
                        {
                            close(fd);
                            return true;
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Can't open camera");
                        }
                    }
                }
                close(fd);
            }
        }
        RCLCPP_ERROR(this->get_logger(), "Can't find OsmoAction4");
        return false;
    }

    void setup_timer()
    {
        double fps = cap_.get(cv::CAP_PROP_FPS);
        if (fps > 0)
        {
            auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / fps));
            timer_ = this->create_wall_timer(period, std::bind(&CameraNode::publish_image, this));
            RCLCPP_INFO(this->get_logger(), "camera fps: %.2f FPS, timer %lld ms", fps, period.count());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Can't get camera fps, use default 100 ms");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraNode::publish_image, this));
        }
    }

    void publish_image()
    {
        cv::Mat frame;
        if (cap_.read(frame))
        {
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            pub_.publish(msg);
        }
    }

    cv::VideoCapture cap_;
    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_send_fps_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}    