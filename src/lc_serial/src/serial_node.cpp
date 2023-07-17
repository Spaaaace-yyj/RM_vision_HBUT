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

  // 是否预测、是否pitch增益
  is_track = declare_parameter("is_track", true);
  is_pitch_gain = declare_parameter("is_pitch_gain", true);
  shoot_speed_ = declare_parameter("shoot_speed", 15.0);
  shoot_delay_ = declare_parameter("shoot_delay", 0.4);
  shoot_delay_spin_ = declare_parameter("shoot_delay_spin_", 0.2);
  gimbal_delay_ = declare_parameter("gimbal_delay", 0.1);
  max_move_yaw_ = declare_parameter("max_move_yaw", 0.0);
  fire_angle_threshold_ = declare_parameter("fire_angle_threshold", 10.0);
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);


  z_gain = declare_parameter("z_gain", 0.0);
  y_gain = declare_parameter("y_gain", 0.0);
  x_gain = declare_parameter("x_gain", 0.0);
  pitch_gain_factor_ = declare_parameter("pitch_gain_factor", 1.0);

  // 构造位姿解算器, 定义参数
  int max_iter = declare_parameter("max_iter", 10);
  float stop_error = declare_parameter("stop_error", 0.001);
  int R_K_iter = declare_parameter("R_K_iter", 50);
  coord_solver_ = std::make_unique<CoordSolver>(max_iter, stop_error, R_K_iter);

  // Create Publisher
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

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
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
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
void SerialDriver::sendData(auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  try
  {
		static rclcpp::Time last_time = this->now();
		static int fps_tmp = 0;
    static int fps = 0;
		auto start_time = this->now();

		if((start_time - last_time).seconds() < 1){
			fps_tmp++;
		}
		else{
			fps = fps_tmp;
			RCLCPP_INFO(rclcpp::get_logger("lc_serial"), "lc_serial send_data FPS: %d", fps);
			fps_tmp = 0;
			last_time = start_time;
		}

    position_marker_.header = msg->header;
    // 未在跟踪状态，或是时间超过间隔，不发送数据
    if (msg->tracking == false)
    {
      position_marker_.action = visualization_msgs::msg::Marker::DELETE;
      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.emplace_back(position_marker_);
      marker_pub_->publish(marker_array);
      return;
    }

    double yaw = msg->yaw, r1 = msg->radius_1, r2 = msg->radius_2;
    double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
    double zc = za + za / 2;
    double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;
    double dz = msg->dz;
    double v_yaw = msg->v_yaw;
    double armor_witch = msg->type == "large" ? 0.225 : 0.135 ;
    size_t a_n = msg->armors_num;

    z_gain = get_parameter("z_gain").as_double();
    y_gain = get_parameter("y_gain").as_double();
    x_gain = get_parameter("x_gain").as_double();

    za += z_gain;
    yc += y_gain;
    xc += x_gain;

    // yc-=0.3;
    // 整车坐标
    geometry_msgs::msg::Point point_c;
    point_c.x = xc;
    point_c.y = yc;
    point_c.z = za + dz / 2;

    // 整车速度
    geometry_msgs::msg::Point velocity_c;
    velocity_c.x = vx;
    velocity_c.y = vy;
    velocity_c.z = vz;

    // 整车角速度
    geometry_msgs::msg::Point angular_v_c;
    // 只是为了方便调试，在rviz2中显示，实际上只用到了 z 轴的角速度
    angular_v_c.x = xc;
    angular_v_c.y = yc;
    // angular_v_c.z = v_yaw / M_PI;
    angular_v_c.z = v_yaw;

    //装甲板坐标
    bool is_current_pair = true;
    std::vector<geometry_msgs::msg::Point> points_a;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);
      points_a.push_back(p_a);
    }

    // 子弹飞行速度为 15m/s, 发单延迟为 0.1s
    shoot_speed_ = get_parameter("shoot_speed").as_double();
    shoot_delay_ = get_parameter("shoot_delay").as_double();
    shoot_delay_spin_ = get_parameter("shoot_delay_spin_").as_double();

    // 子弹飞行时间加上发单延迟
    double delay_translation = shoot_delay_ + sqrt(xc*xc + yc*yc + zc*zc) / shoot_speed_;

    // 整车预测坐标
    geometry_msgs::msg::Point point_c_pre;
    point_c_pre.x = point_c.x + velocity_c.x * delay_translation;
    point_c_pre.y = point_c.y + velocity_c.y * delay_translation;
    point_c_pre.z = point_c.z + velocity_c.z * delay_translation;

    // 整车角度预测
    // TODO: 角度预测时间要短些
    double delay_spin = shoot_delay_spin_ + sqrt(xc*xc + yc*yc + zc*zc) / shoot_speed_;
    double yaw_pre = yaw + angular_v_c.z * delay_spin;
    
    //装甲板坐标预测
    is_current_pair = true;
    std::vector<geometry_msgs::msg::Point> points_a_pre;
    r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw_pre + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = point_c_pre.x - r * cos(tmp_yaw);
      p_a.y = point_c_pre.y - r * sin(tmp_yaw);
      points_a_pre.push_back(p_a);
    }

    // // 匹配最优装甲板
    // // 按照v_yaw，优先选择最接近面朝摄像头的装甲板，面朝摄像头的装甲板的yaw为0，但需要考虑一定的阈值
    // // 如果最接近0的yaw大于另一个阈值，则认为没有最优装甲板，不进行射击
    double target_yaw = yaw;
    if(is_track){
      // 由于云台转动的延迟，进行最优装甲板筛选时，多预测一点，这里的delay应该比上面的delay大
      target_yaw += angular_v_c.z * (delay_spin + gimbal_delay_);
    }

    int index = 0;
    double min_yaw = 2 * M_PI;
    double c_yaw = atan2(point_c.y, point_c.x);
    // RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "c_yaw: %f", c_ycaw * 180 / M_PI);
    for (size_t i = 0; i < a_n; i++)
    {
      double tmp_yaw = target_yaw + i * (2 * M_PI / a_n);
      // if(i==0)
        // RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "%ld : yaw: %f", i, tmp_yaw * 180 / M_PI);
      tmp_yaw = std::fmod(tmp_yaw + 2 * M_PI, 2 * M_PI);
      double delta_to_0 = std::fabs(tmp_yaw - c_yaw);
      double delta_to_2pi = std::fabs(tmp_yaw - (c_yaw + 2 * M_PI));
      double delta_to_zero = std::min(delta_to_0, delta_to_2pi);
      // RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "%ld : delta_to_zero: %f", i, delta_to_zero * 180 / M_PI);
      if (delta_to_zero < min_yaw)
      {
        min_yaw = delta_to_zero;
        index = i;
      }
    }
    
    // RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "now min yaw: %f", min_yaw * 180 / M_PI);
    // RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "now min gimbal_yaw_: %f", (-gimbal_yaw_) * 180 / M_PI);

    // 如果最优装甲板的yaw大于阈值，则认为没有最优装甲板，不进行射击
    if(min_yaw > max_move_yaw_ * M_PI / 180){
      RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "No optimal armor, now min yaw: %f", min_yaw * 180 / M_PI);
      return ;
    }

    // 按照距离最近确定最优装甲板，已废弃
    // int index = 0;
    // double min = 1000;
    // for (size_t i = 0; i < a_n; i++)
    // {
    //   double x_p = points_a[i].x;
    //   double y_p = points_a[i].y;
    //   if(is_track){
    //     x_p = points_a_pre[i].x;
    //     y_p = points_a_pre[i].y;
    //   }
    //   double tmp_dis = sqrt(x_p*x_p + y_p*y_p);
    //   double tmp = fabs(tmp_dis);
    //   if (tmp < min)
    //   {
    //     min = tmp;
    //     index = i;
    //   }
    // }

    // 对最优装甲板进行位姿解算
    // index = 0;
    double x, y, z;
    if(is_track){
      x = points_a_pre[index].x;
      y = points_a_pre[index].y;
      z = points_a_pre[index].z;
    }else{
      x = points_a[index].x;
      y = points_a[index].y;
      z = points_a[index].z;
    }

    // 位姿解算
    double send_pitch = atan2(z, sqrt(x * x + y * y));
    double send_yaw = -atan2(y, x);
    double send_is_fire = 0;

    // 对抬枪角度进行增益
    if(is_pitch_gain){
      pitch_gain_factor_ = get_parameter("pitch_gain_factor").as_double();
      coord_solver_->bullet_speed = shoot_speed_;
      Eigen::Vector3d xyz(x, y, z);
      double send_pitch_gain = coord_solver_->dynamicCalcPitchOffset(xyz);
      // RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "send_pitch_gain 111:%lf", send_pitch_gain);
      send_pitch_gain = send_pitch_gain * M_PI / 180.0;
      // RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "send_pitch_gain 222:%lf", send_pitch_gain);
      if(pitch_gain_factor_ > 10.0)
        send_pitch_gain *= xyz.norm() * (pitch_gain_factor_ - 10.0);
      else
        send_pitch_gain *= pitch_gain_factor_;
      send_pitch += send_pitch_gain;
    }

    Eigen::Vector3d xyz(x, y, z);
    double shoot_diff = atan((armor_witch/2) / xyz.norm());
    RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver shoot_diff: %f", shoot_diff);
    static int loss_cnt = 0;
    //如果云台yaw、pitch与当前目标yaw、pitch的差值小于阈值，则认为云台已经对准目标，可以进行射击
    if( std::fabs(send_yaw - gimbal_yaw_) < fire_angle_threshold_ * shoot_diff && 
        std::fabs(send_pitch - gimbal_pitch_) < fire_angle_threshold_ * shoot_diff )
    {
      send_is_fire = 1.0;
      loss_cnt = 0;
    }else 
    {
      loss_cnt ++;
      if(loss_cnt >= 4)
        send_is_fire = 0.0;
      else 
        send_is_fire = 1.0;
    }
    // 发布marker
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = x;
    position_marker_.pose.position.y = y;
    position_marker_.pose.position.z = z;
  
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.emplace_back(position_marker_);
    marker_pub_->publish(marker_array);

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
    // RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver sending data: %s", data.data());
    // RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver sending data: %d", str_len);
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
      // std::cout<<"111"<<std::endl;
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
        // std::cout<<"222"<<std::endl;
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
            // RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver receiving data: %s", data.data());
            imu_yaw = cJSON_GetObjectItem(dat, "imu_yaw")->valuedouble;
            // std::cout<<"222"<<std::endl;
            imu_pitch = cJSON_GetObjectItem(dat, "imu_pitch")->valuedouble;
            // std::cout<<"222"<<std::endl;
            // robot_color = (int)(cJSON_GetObjectItem(dat, "robot_color")->valuedouble);
            // robot_level = (int)(cJSON_GetObjectItem(dat, "speed_level")->valuedouble);
          }
        }
        // if (!initial_set_param_ || robot_color != previous_receive_color_) {
        //   RCLCPP_INFO(get_logger(), "Setting detect_color to %d...", robot_color);
        //   setParam(rclcpp::Parameter("detect_color", robot_color));
        //   previous_receive_color_ = robot_color;
        // }
        // if(!initial_set_param_) {
        //   double speed=0.0;
        //   if(robot_level==0)
        //     speed=13.6;
        //   else if(robot_level==1)
        //     speed=16.7;
        //   else if(robot_level==2)
        //     speed=28.0;
        //   else
        //     speed=13.6;
        //   RCLCPP_INFO(get_logger(), "Setting detect_speed to %d...", robot_level);

        //   setParam(rclcpp::Parameter("shoot_speed", speed));
        //   previous_receive_speed_ = robot_level;
        // }



        // 收到电控数据
        RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver receiving data: %s", data.data());
        if (std::isnan(imu_yaw) || std::isnan(imu_pitch))
          continue;
        try
        {
          // 保存云台角度
          gimbal_yaw_ = -imu_yaw;
          gimbal_pitch_ = -imu_pitch;
          sensor_msgs::msg::JointState joint_state;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          joint_state.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");
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