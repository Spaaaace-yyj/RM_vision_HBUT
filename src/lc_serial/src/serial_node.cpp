/**
 * @file serial_node.cpp
 * @brief
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-03-04
 *
 */

#include "lc_serial/serial_node.h"

SerialDriver::SerialDriver(const rclcpp::NodeOptions& options)
  : Node("lc_serial_driver", options)
  , owned_ctx_{ new IoContext(2) }
  , serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }
{
  RCLCPP_INFO(rclcpp::get_logger("lc_serial"), "Start SerialDriver!");

  getParams();
  // Create Publisher
  debug_serial_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugSerial>(
    "/debug/auto_aim_debug_data", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/armor_debug_serial/marker_array", 10);
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
  gimbal_feed_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalFeed>("/gimbal_feed", 10);
  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/armor_detector");
  // serial_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/lc_serial_driver");
  try
  {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open())
    {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&SerialDriver::receiveData, this);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error creating lc_serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
  
  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.2;
  position_marker_.color.a = 1.0;
  position_marker_.color.r = 1.0;
  position_marker_.color.b = 1.0;
  
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/lc_serial/marker", 10);

  // Create Subscription
  gimbal_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalControl>(
      "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&SerialDriver::sendData, this, std::placeholders::_1));

}

SerialDriver::~SerialDriver()
{
  if (receive_thread_.joinable())
  {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open())
  {
    serial_driver_->port()->close();
  }

  if (owned_ctx_)
  {
    owned_ctx_->waitForExit();
  }
}

/**
 * @brief 发送数据
 * @param msg
 */

void SerialDriver::sendData(auto_aim_interfaces::msg::GimbalControl::SharedPtr msg)
{
  try
  {
    double send_yaw = msg->yaw;
    double send_pitch = msg->pitch;
    double send_is_fire = msg->is_fire;

    /* 创建一个JSON数据对象(链表头结点) */
    char* str = NULL;

    //"date":[x,y];
    cJSON* cjson_date = cJSON_CreateArray();
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(send_yaw));
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(send_pitch));
    cJSON_AddItemToArray(cjson_date, cJSON_CreateNumber(send_is_fire));

    // dat
    cJSON* cjson_dat = cJSON_CreateObject();
    cJSON_AddItemToObject(cjson_dat, "date", cjson_date);
    cJSON_AddStringToObject(cjson_dat, "mode", "visual");

    // send
    cJSON* cjson_send = cJSON_CreateObject();
    cJSON_AddStringToObject(cjson_send, "cmd", "ctr_mode");

    cJSON_AddItemToObject(cjson_send, "dat", cjson_dat);

    // 转化为待发送数据结构
    str = cJSON_PrintUnformatted(cjson_send);
    cJSON_Delete(cjson_send);
    int str_len = std::strlen(str);
    
    std::vector<uint8_t> data(str, str + str_len);
    data.push_back('\n');
    // data.push_back('\0');

    serial_driver_->port()->send(data);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

  /**
   * @brief 不断接收电控数据
   */
  void SerialDriver::receiveData()
  {
    std::vector<uint8_t> data;
    while (rclcpp::ok())
    {
      try
      {

        data.clear();
        data.resize(200);
        int rec_len = serial_driver_->port()->receive(data);
        if (rec_len >= 150 || rec_len <= 10)
          continue;

        data[rec_len - 1] = '\0';
        double imu_yaw = 0, imu_pitch = 0;
        // int robot_color = -1;
        // int robot_level = -1;//0:13.6
        //1:16.7
        //2:28.0
        // 解析json
        cJSON* root = cJSON_Parse((char*)data.data());
        if (!root)
        {
          RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "receiveData Error before: [%s]", cJSON_GetErrorPtr());
          continue;
        }
        else
        {
          cJSON* dat = cJSON_GetObjectItem(root, "dat");
          if (!dat)
          {
            RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "receiveData Error before: [%s]", cJSON_GetErrorPtr());
            continue;
          }
          else
          {
            imu_yaw = cJSON_GetObjectItem(dat, "imu_yaw")->valuedouble;

            imu_pitch = cJSON_GetObjectItem(dat, "imu_pitch")->valuedouble;

          }
        }
        // 收到电控数据
        RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver receiving data: %s", data.data());
        if (std::isnan(imu_yaw) || std::isnan(imu_pitch))
          continue;
        try
        {
          // 保存云台角度
          auto_aim_interfaces::msg::GimbalFeed gimbal_feed_msg;
          gimbal_feed_msg.pitch = imu_pitch;
          gimbal_feed_msg.yaw = imu_yaw;
          gimbal_feed_pub_->publish(gimbal_feed_msg);

          sensor_msgs::msg::JointState joint_state;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          joint_state.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("gimbal_pitch_joint");
          joint_state.name.push_back("gimbal_yaw_joint");
          joint_state.position.push_back(imu_pitch);
          joint_state.position.push_back(imu_yaw);
          joint_state_pub_->publish(joint_state);
        }
        catch (const std::exception& ex)
        {
          RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error while receiving data: %s", ex.what());
        }
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error while receiving data: %s", ex.what());
        reopenPort();      
      }
    }
  }

  /**
   * @brief 重启串口
   */
  void SerialDriver::reopenPort()
  {
    RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "Attempting to reopen port");
    try
    {
      if (serial_driver_->port()->is_open())
      {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(rclcpp::get_logger("lc_serial"), "Successfully reopened port");
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error while reopening port: %s", ex.what());
      if (rclcpp::ok())
      {
        rclcpp::sleep_for(std::chrono::seconds(1));
        reopenPort();
      }
    }
  }

void SerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try
  {
    device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "The device name provided was invalid");
    throw ex;
  }

  try
  {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "The baud_rate provided was invalid");
    throw ex;
  }

  try
  {
    const auto fc_string = declare_parameter<std::string>("flow_control", "none");

    if (fc_string == "none")
    {
      fc = FlowControl::NONE;
    }
    else if (fc_string == "hardware")
    {
      fc = FlowControl::HARDWARE;
    }
    else if (fc_string == "software")
    {
      fc = FlowControl::SOFTWARE;
    }
    else
    {
      throw std::invalid_argument{ "The flow_control parameter must be one of: none, software, or hardware." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "The flow_control provided was invalid");
    throw ex;
  }

  try
  {
    const auto pt_string = declare_parameter<std::string>("parity", "none");

    if (pt_string == "none")
    {
      pt = Parity::NONE;
    }
    else if (pt_string == "odd")
    {
      pt = Parity::ODD;
    }
    else if (pt_string == "even")
    {
      pt = Parity::EVEN;
    }
    else
    {
      throw std::invalid_argument{ "The parity parameter must be one of: none, odd, or even." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "The parity provided was invalid");
    throw ex;
  }

  try
  {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "1");

    if (sb_string == "1" || sb_string == "1.0")
    {
      sb = StopBits::ONE;
    }
    else if (sb_string == "1.5")
    {
      sb = StopBits::ONE_POINT_FIVE;
    }
    else if (sb_string == "2" || sb_string == "2.0")
    {
      sb = StopBits::TWO;
    }
    else
    {
      throw std::invalid_argument{ "The stop_bits parameter must be one of: 1, 1.5, or 2." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}


void SerialDriver::setParam(const rclcpp::Parameter & param)
{
  try
  {
    if (detector_param_client_->service_is_ready()) {
      detector_param_client_->set_parameters(
        {param},
        [this, param](
          const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> & results) {
          for (const auto & result : results.get()) {
            if (!result.successful) {
              RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
              return;
            }
          }
          RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
          initial_set_param_ = true;
        });
    } else {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "setParam Error");
    throw ex;
  }
}

#include "rclcpp_components/register_node_macro.hpp"

  // Register the component with class_loader.
  // This acts as a sort of entry point, allowing the component to be discoverable when its library
  // is being loaded into a running process.
  RCLCPP_COMPONENTS_REGISTER_NODE(SerialDriver)