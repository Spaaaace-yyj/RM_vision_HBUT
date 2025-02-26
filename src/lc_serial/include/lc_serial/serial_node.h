/**
 * @file serial_node.h
 * @brief 串口
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-04
 * 
 */

#ifndef SERIAL__SERIAL_NODE_H_
#define SERIAL__SERIAL_NODE_H_

// c++
#include <vector>
#include <cmath>

// ros
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial_driver/serial_driver.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// user
#include "auto_aim_interfaces/msg/target.hpp"
#include <lc_serial/cJSON.h>
//buff
#include <buff_interfaces/msg/rune.hpp>
#include <buff_interfaces/msg/rune_info.hpp>
// #include <lc_serial/../../src/cJSON.c>
#include <lc_serial/malloc.h>
#include <lc_serial/coordsolver.h>
// #include <lc_serial/../../src/coordsolver.cpp>

class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions & options);

  ~SerialDriver() override;

private:
  void getParams();

  void receiveData();
  
  void getArmorInfo(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void getRuneInfo(buff_interfaces::msg::Rune msg);

  void sendData(const double send_pitch, const double send_yaw, const int send_is_fire);
  
  void taskSelection(std::string task_mode);

  void reopenPort();

  std::unique_ptr<CoordSolver> coord_solver_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<buff_interfaces::msg::Rune>::SharedPtr Rune_target_sub_;

  void setParam(const rclcpp::Parameter & param);
  bool initial_set_param_ = false;
  bool initial_set_param_2 = false;
  uint8_tc previous_receive_color_ = 0;
  uint8_tc previous_receive_speed_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  rclcpp::AsyncParametersClient::SharedPtr serial_param_client_;
  

  std::thread receive_thread_;

  double shoot_speed_;
  double shoot_delay_;
  double shoot_delay_spin_;

  // 云台的延迟，单位为秒，云台转动需要时间，目前只在选择最优装甲板时使用，用于计算更加提前的装甲板的位置
  double gimbal_delay_;

  // 最优装甲板的最大角度，大于这个角度就不击打，单位为度
  double max_move_yaw_;

  // 开火角度阈值，yaw、pitch的偏差小于这个角度就开火，单位为度
  double fire_angle_threshold_;

  // 云台的yaw、pitch的偏差，用于判断是否开火
  double gimbal_yaw_;
  double gimbal_pitch_;

  // 目标点的x、y、z的静态补偿
  double z_gain;
  double y_gain;
  double x_gain;
  //目标能量机关的x、y、z的静态补偿
  double rune_z_gain;
  double rune_y_gain;
  double rune_x_gain;

  // pitch 动态补偿系数
  double pitch_gain_factor_;

  // 时间戳偏移量
  double timestamp_offset_ = 0;

  bool is_track;
  bool is_pitch_gain;


  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

};

#endif  // SERIAL__SERIAL_NODE_H_