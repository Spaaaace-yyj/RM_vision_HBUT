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

// serial_driver
#include <serial_driver/serial_driver.hpp>

#define VISION_SEND_SIZE 36u

typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

//按照通信格式修改接收或发送结构体，保证和下位机通信一致
typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch;
	float yaw;
} Vision_Send_s;

typedef struct
{
    Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

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
	void navigation_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
    std::vector<uint8_t> buffer;


    Vision_Recv_s recv_data;
	Vision_Send_s send_data;

private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	geometry_msgs::msg::Twist latest_cmd_vel_;

private:

    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    std::thread receive_thread_;

};

#endif