#include "../include/lc_serial_test/lc_serial_test.hpp"

LcSerialTestNode::LcSerialTestNode(const rclcpp::NodeOptions & options) 
        : Node("serial_test",  options)
        , owned_ctx_{ new IoContext(2) }
        , serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }
{

    RCLCPP_INFO(this->get_logger(), "Starting serial node...");

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/red_standard_robot1/cmd_vel", 1, std::bind(&LcSerialTestNode::navigation_callback, this, std::placeholders::_1));

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
        throw ex;
    }

}

void LcSerialTestNode::navigation_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "(%f, %f, %f)", msg->linear.x, msg->linear.y, msg->linear.z);
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
    // std::cout << "decode result:" << recv_data.yaw << ", " << recv_data.pitch << ", " << recv_data.row << std::endl;
}

void LcSerialTestNode::SendData(){
    uint16_t flags_register;
    uint16_t tx_len;
    uint8_t send_temp[64]= {0};

    flags_register = 0x0001;

    get_protocol_send_data(0x01, flags_register, &send_data.pitch, 2, send_temp, &tx_len);
    std::vector<uint8_t> send_buffer(send_temp, send_temp + tx_len);
    try {
        if(serial_driver_->port()->is_open()){
            serial_driver_->port()->send(send_buffer);

            RCLCPP_INFO(this->get_logger(), "Send %d bytes", tx_len);
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

        }catch (const std::exception & e){
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            OpenPort();
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
    }
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
