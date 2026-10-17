#ifndef LC_SERIAL_TEST_HPP
#define LC_SERIAL_TEST_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <auto_aim_interfaces/msg/gimbal_control.hpp>
#include <auto_aim_interfaces/msg/gimbal_feed.hpp>

// serial_driver
#include <serial_driver/serial_driver.hpp>

// 协议层
#include "lc_rm_serial_wlz/include/test/protocol.hpp"
#include "lc_rm_serial_wlz/include/test/serial_process.hpp"

namespace lc_serial
{

    class LcSerialTestNode : public rclcpp::Node
    {
        public:
            explicit LcSerialTestNode(const rclcpp::NodeOptions& options);
            ~LcSerialTestNode() override;

        private:
                // ---------------- ROS 回调 ----------------
                void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
                void onGimbalControl(
                        const auto_aim_interfaces::msg::GimbalControl::SharedPtr msg);

                // ---------------- 收发 ----------------
                void sendLoop();
                void receiveLoop();

                // ---------------- 串口 ----------------
                bool openPort();
                void closePort();

        private:
                // 串口设备
                std::string device_name_{"/dev/ttyACM0"};

                std::unique_ptr<drivers::common::IoContext> owned_ctx_;
                std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
                std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;

                std::atomic<bool> running_{false};
                std::thread receive_thread_;
                std::vector<uint8_t> rx_buffer_;

                //锁
                std::mutex serial_mutex_;//保护串口 open/close/send
                std::mutex data_mutex_;//保护tx_data_/tx_cmd_id_/rx_frame_

                //协议数据
                TxData tx_data_{};
                uint16_t chassis_cmd_ = static_cast<uint16_t>(ChassisCmd::FOLLOWING); // 默认跟随
                uint16_t gimbal_cmd_  = static_cast<uint16_t>(GimbalCmd::ENABLE);     // 默认使能
                uint16_t shoot_cmd_   = static_cast<uint16_t>(ShootCmd::OFF);         // 默认关闭
                RxData rx_frame_{};

                //ros接口
                rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
                rclcpp::Subscription<auto_aim_interfaces::msg::GimbalControl>::SharedPtr 
                gimbal_control_sub_;
                rclcpp::Publisher<auto_aim_interfaces::msg::GimbalFeed>::SharedPtr
                gimbal_feed_pub_;
                rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
                rclcpp::TimerBase::SharedPtr serial_timer_;

                //方向反转参数
                bool send_reverse_pitch_ = false;
                bool send_reverse_yaw_ = true;
                bool recv_reverse_pitch_ = false;
                bool recv_reverse_yaw_ = true;
                bool recv_tf_reverse_pitch_ = true;
                bool recv_tf_reverse_yaw_ = false;

                

    };
}

#endif  // LC_SERIAL_TEST_HPP