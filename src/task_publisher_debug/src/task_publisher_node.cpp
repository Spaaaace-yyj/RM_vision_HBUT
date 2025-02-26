#include<iostream>
#include"rclcpp/rclcpp.hpp"
#include"std_msgs/msg/string.hpp"

using namespace std;

class TaskPublisher : public rclcpp::Node
{
    public:
        TaskPublisher() : Node("task_publisher_debug")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("/task_mode", 10);
            timer_ = this->create_wall_timer(0.1s, std::bind(&TaskPublisher::TimerCallback, this));
            this->declare_parameter("task_mode", 0);
            this->get_logger().set_level(rclcpp::Logger::Level::Debug);
            RCLCPP_INFO(this->get_logger(), "Task_debug publisher initialized");

        }
    private:

        void TimerCallback(){
            int task_mode_ = this->get_parameter("task_mode").as_int();
            cout << "task_mode: " << task_mode_ << endl;
            if(task_mode_ == 0){
                task_mode_str_ = "small_buff";
            }else if (task_mode_ == 1){
                task_mode_str_ = "large_buff";
            }else{
                task_mode_str_ = "none";
            }
            auto message = std_msgs::msg::String();
            message.data = task_mode_str_;
            publisher_->publish(message);
            RCLCPP_DEBUG(this->get_logger(), "publish task: '%s'", message.data.c_str());
        }
        std::string task_mode_str_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskPublisher>());
    rclcpp::shutdown();
    return 0;
}
