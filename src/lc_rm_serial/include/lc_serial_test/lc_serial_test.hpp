#ifndef LC_SERTAL_TEST_HPP
#define LC_SERIAL_TEST_HPP

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iomanip>
#include <iostream>
//USER
#include "serial_process.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "auto_aim_interfaces/msg/gimbal_feed.hpp"
#include "auto_aim_interfaces/msg/gimbal_control.hpp"
// serial_driver
#include <serial_driver/serial_driver.hpp>

#define VISION_SEND_SIZE 36u

//16位标志位定义结构体
#define CAN_FIRE_BIT 0
#define	TRACING_STATE_BIT 1

//按照通信格式修改接收或发送结构体，保证和下位机通信一致，只有浮点位
typedef struct
{
	bool fire = false;
	bool tracing = false;

	float pitch;
	float yaw;
	float is_fire;
	float v_x;
	float v_y;
	float w;
} Vision_Send_s;

typedef struct
{
	float yaw;
	float pitch;
    float row;
} Vision_Recv_s;

class LcSerialTestNode : public rclcpp::Node
{
public:
    explicit LcSerialTestNode(const rclcpp::NodeOptions & options);

    ~LcSerialTestNode();
private:
    void receiveLoop();

    void DecodeData();

    void SendData();

	void OpenPort();
private:
	void NavigationCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

	void GimbalControlCallback(const auto_aim_interfaces::msg::GimbalControl::SharedPtr msg);

	inline void setBit(uint16_t &data, uint8_t n, bool state);

	inline bool getBit(uint16_t &data, uint8_t n);

private:
    std::vector<uint8_t> buffer;


    Vision_Recv_s recv_data;
	Vision_Send_s send_data = {
		.pitch = 0.0,
		.yaw = 0.0,
		.v_x = 0.0,
		.v_y = 0.0,
		.w = 0.0,
	};

private:
	//subscription
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	rclcpp::Subscription<auto_aim_interfaces::msg::GimbalControl>::SharedPtr gimbal_control_sub_;
	//publisher
	rclcpp::Publisher<auto_aim_interfaces::msg::GimbalFeed>::SharedPtr gimbal_feed_pub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

	geometry_msgs::msg::Twist latest_cmd_vel_;

private:
  	rclcpp::TimerBase::SharedPtr serial_timer_;

    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    std::thread receive_thread_;

private:
	bool send_reverse_pitch = false;
	bool send_reverse_yaw = true;
	bool recv_reverse_pitch = false;
	bool recv_reverse_yaw = true;
	bool recv_tf_reverse_pitch = true;
	bool recv_tf_reverse_yaw = false;
};

#endif