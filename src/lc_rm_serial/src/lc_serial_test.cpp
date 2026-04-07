#include "../include/lc_serial_test/lc_serial_test.hpp"

LcSerialTestNode::LcSerialTestNode(const rclcpp::NodeOptions & options) 
        : Node("serial_test",  options)
        , owned_ctx_{ new IoContext(2) }
        , serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }
{

    RCLCPP_INFO(this->get_logger(), "Starting serial node...");
    //subscription
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, std::bind(&LcSerialTestNode::NavigationCallback, this, std::placeholders::_1));
    gimbal_control_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalControl>(
        "control/gimbal_control", 1, std::bind(&LcSerialTestNode::GimbalControlCallback, this, std::placeholders::_1));
    //publisher
    gimbal_feed_pub_ =
        this->create_publisher<auto_aim_interfaces::msg::GimbalFeed>("/gimbal_feed", 10);
    joint_state_pub_ =
          this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
    
    serial_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
        std::bind(&LcSerialTestNode::SendData, this));

    
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate = 921600;
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;
    
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>
        (baud_rate, fc, pt, sb);
    
    try{
        serial_driver_->init_port("/dev/ttyACM0", *device_config_);
        if(!serial_driver_->port()->is_open()){
            serial_driver_->port()->open();
            receive_thread_ = std::thread(&LcSerialTestNode::receiveLoop, this);
        }
    }catch (const std::exception& ex){
        RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error creating lc_serial port: %s - %s", device_name_.c_str(), ex.what());
        // throw ex;
    }

}

void LcSerialTestNode::NavigationCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "navigation_msg(%f, %f, %f)", msg->linear.x, msg->linear.y, msg->linear.z);
    send_data.v_x = -msg->linear.y * 10;
    send_data.v_y = msg->linear.x * 10;
    send_data.w = msg->linear.z;
}

void LcSerialTestNode::GimbalControlCallback(const auto_aim_interfaces::msg::GimbalControl::SharedPtr msg)
{
    float send_pitch = msg->pitch * (180.0 / M_PI);
    float send_yaw = msg->yaw * (180.0 / M_PI);
    if (send_reverse_pitch) send_pitch *= -1;
    if (send_reverse_yaw) send_yaw *= -1;

    send_data.pitch = send_pitch;
    send_data.yaw = send_yaw;
    send_data.fire = static_cast<bool>(msg->is_fire);
    send_data.is_fire = msg->is_fire;
    send_data.tracing = static_cast<bool>(msg->tracing);
    // SendData();
}

void LcSerialTestNode::OpenPort(){
    try
    {
        if (serial_driver_->port()->is_open())
        {
            RCLCPP_WARN(this->get_logger(), "Serial port is open, closing...");
            serial_driver_->port()->close();
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        serial_driver_->port()->open();

        RCLCPP_INFO(
            this->get_logger(),
            "Serial port reopened successfully"
        );
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Serial port reopen failed: %s",
            e.what()
        );
    }
}

void LcSerialTestNode::DecodeData(){
    uint16_t flags_register;
    get_protocol_info(buffer.data(), &flags_register, (uint8_t *)&recv_data.yaw);
}

void LcSerialTestNode::SendData(){
    uint16_t flags_register = 0x0000;
    uint16_t tx_len;
    uint8_t send_temp[64]= {0};
    //标志位处理
    setBit(flags_register, CAN_FIRE_BIT, send_data.fire);
    setBit(flags_register, TRACING_STATE_BIT, send_data.tracing);

    get_protocol_send_data(0x01, flags_register, &send_data.pitch, 5, send_temp, &tx_len);
    std::vector<uint8_t> send_buffer(send_temp, send_temp + tx_len);
    //debug
    // for (size_t i = 0; i < send_buffer.size(); ++i)
    // {
    //     printf("%02X ", send_buffer[i]);
    // }
    //
    // printf("\n");
    try {
        if(serial_driver_->port()->is_open()){
            serial_driver_->port()->send(send_buffer);

            RCLCPP_DEBUG(this->get_logger(), "Send %d bytes", tx_len);
        }else{
            RCLCPP_ERROR(this->get_logger(), "Disconnect with Serial port! Try to reconnect....");
            OpenPort();
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Serial send error: %s", e.what());
    }
}

void LcSerialTestNode::receiveLoop()
{
    while (rclcpp::ok())
    {
        try
        {
            buffer.clear();
            buffer.resize(256);
            size_t n = serial_driver_->port()->receive(buffer);
            if(n <= 10){
                RCLCPP_WARN(this->get_logger(), "receive data is too short, skip this loop! [%d byte]", n);
                continue;
            };

            if(buffer[0] == 0xA5){
                // std::cout << "receive [" << n << " byte] data : ";
                // for (size_t i = 0; i < n; i++){
                //     std::cout << std::hex
                //     << std::setw(2)
                //     << std::setfill('0')
                //     << static_cast<int>(buffer[i])
                //     << " ";
                // }
                // std::cout << std::dec << std::endl;
                DecodeData();
            }else{
                RCLCPP_WARN(this->get_logger(), "Can't find CMD_ID! skip this loop!");
            }

            float gimbal_pitch = recv_data.pitch * (M_PI / 180.0);
            float gimbal_yaw = recv_data.yaw * (M_PI / 180.0);

            float gimbal_tf_pitch = gimbal_pitch;
            float gimbal_tf_yaw = gimbal_yaw;
            if (recv_tf_reverse_pitch) gimbal_tf_pitch *= -1;
            if (recv_tf_reverse_yaw) gimbal_tf_yaw *= -1;

            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = this->now();
            joint_state.name.push_back("gimbal_pitch_joint");
            joint_state.name.push_back("gimbal_yaw_joint");
            joint_state.position.push_back(gimbal_tf_pitch);
            joint_state.position.push_back(gimbal_tf_yaw);
            joint_state_pub_->publish(joint_state);

            if (recv_reverse_pitch) gimbal_pitch *= -1;
            if (recv_reverse_yaw) gimbal_yaw *= -1;

            auto_aim_interfaces::msg::GimbalFeed gimbal_feed_msg;
            gimbal_feed_msg.pitch = gimbal_pitch;
            gimbal_feed_msg.yaw = gimbal_yaw;
            gimbal_feed_pub_->publish(gimbal_feed_msg);

        }catch (const std::exception & e){
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            OpenPort();
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
    }
}

void LcSerialTestNode::setBit(uint16_t& data, uint8_t n, bool state)
{
    if (n >= 16) return;
    if (state)
        data |= (1U << n);
    else
        data &= ~(1U << n);
}

bool LcSerialTestNode::getBit(uint16_t& data, uint8_t n)
{
    return (data >> n) & 1U;
}

LcSerialTestNode::~LcSerialTestNode()
{
    if (receive_thread_.joinable())
    {
        receive_thread_.join();
    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LcSerialTestNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
