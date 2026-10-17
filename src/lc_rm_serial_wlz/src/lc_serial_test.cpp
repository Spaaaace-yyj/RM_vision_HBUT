#include "lc_rm_serial_wlz/include/test/lc_serial_test.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;   

namespace lc_serial
{
    //构造
    LcSerialTestNode::LcSerialTestNode(const rclcpp::NodeOptions& options)
        : Node("serial_test", options)
        {
            RCLCPP_INFO(get_logger(), "Starting lc_serial_test node...");

            //参数
            device_name_           = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
            send_reverse_pitch_    = declare_parameter<bool>("send_reverse_pitch", false);
            send_reverse_yaw_      = declare_parameter<bool>("send_reverse_yaw", true);
            recv_reverse_pitch_    = declare_parameter<bool>("recv_reverse_pitch", false);
            recv_reverse_yaw_      = declare_parameter<bool>("recv_reverse_yaw", true);
            recv_tf_reverse_pitch_ = declare_parameter<bool>("recv_tf_reverse_pitch", true);
            recv_tf_reverse_yaw_   = declare_parameter<bool>("recv_tf_reverse_yaw", false);

            //订阅
            cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 1, std::bind(&LcSerialTestNode::onCmdVel,this,std::placeholders::_1));
            
            gimbal_control_sub_ = create_subscription<auto_aim_interfaces::msg::GimbalControl>(
                "control/gimbal_control", 1, std::bind(&LcSerialTestNode::onGimbalControl,this,std::placeholders::_1));
            
            //发布
            gimbal_feed_pub_ = create_publisher<auto_aim_interfaces::msg::GimbalFeed>(
                "/gimbal_feed", 10
            );
            joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10
            );

            //串口对象
            owned_ctx_ = std::make_unique<drivers::common::IoContext>(2);
            serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
            device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
                921600,
                drivers::serial_driver::FlowControl::NONE,
                drivers::serial_driver::Parity::NONE,
                drivers::serial_driver::StopBits::ONE 
            );

            //打开串口+启动线程
            running_ = true;
            if(!openPort())
            {
                RCLCPP_ERROR(get_logger(), "Failed to open serial port at startup.Will retry in sendLoop().");
            }
            else
            {
                
                receive_thread_ = std::thread(&LcSerialTestNode::receiveLoop, this);
            }

            //定时发送（100Hz）
            serial_timer_ = create_wall_timer(10ms, std::bind(&LcSerialTestNode::sendLoop, this));

        }

        //析构
        LcSerialTestNode::~LcSerialTestNode()
        {
            running_ = false;
            closePort();
            if(receive_thread_.joinable())
            {
                receive_thread_.join();
            }
        }

        //串口开关
        bool LcSerialTestNode::openPort()
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            try
            {
                // 1. 必须先初始化端口
                serial_driver_->init_port(device_name_, *device_config_);
                
                // 2. 再判断是否打开
                if(serial_driver_->port() && !serial_driver_->port()->is_open())
                {
                    serial_driver_->port()->open();
                }
                
                RCLCPP_INFO(get_logger(), "Serial port opened: %s", device_name_.c_str());
                return true;
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
                return false;
            }
        }

        void LcSerialTestNode::closePort()
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            try
            {
                if(serial_driver_ && serial_driver_->port()->is_open())
                {
                    serial_driver_->port()->close();
                    RCLCPP_INFO(get_logger(), "Serial port closed.");
                }
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Close port failed: %s", e.what());
            }
        }

        //命令码组装
        uint16_t LcSerialTestNode::BuildCmdId(bool is_fire, bool tracing) const
        {
            uint16_t cmd_id = 0;

            //底盘：默认跟随
            cmd_id |= (static_cast<uint16_t>(ChassisCmd::FOLLOWING) << CHASSIS_CMD_SHIFT);
            //云台：默认使能
            cmd_id |= (static_cast<uint16_t>(GimbalCmd::ENABLE) << GIMBAL_CMD_SHIFT);
            //发射机构：开火 > 摩擦轮 > 关闭
            uint16_t shoot = static_cast<uint16_t>(ShootCmd::OFF);
            if(is_fire)
            {
                shoot = static_cast<uint16_t>(ShootCmd::FIRE);
            }
            else if(tracing)
            {
                shoot = static_cast<uint16_t>(ShootCmd::FICTION_ON);
            }

            cmd_id |= (shoot << SHOOT_CMD_SHIFT);

            return cmd_id;
        }

        //订阅回调
        void LcSerialTestNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            tx_data_.forward = static_cast<float>(msg->linear.x);
            tx_data_.turn    = static_cast<float>(msg->linear.y);
            tx_data_.rotate  = static_cast<float>(msg->angular.z);
        }

        void LcSerialTestNode::onGimbalControl(const auto_aim_interfaces::msg::GimbalControl::SharedPtr msg)
        {
            float yaw_deg = static_cast<float>(msg->yaw * (180.0 / M_PI));
            float pitch_deg = static_cast<float>(msg->pitch * (180.0 / M_PI));

            if(send_reverse_yaw_)
            {
                yaw_deg = -yaw_deg;
            }
            if(send_reverse_pitch_) 
            {
                pitch_deg = -pitch_deg;
            }

            std::lock_guard<std::mutex> lock(data_mutex_);
            tx_data_.yaw = yaw_deg;
            tx_data_.pitch = pitch_deg;
            tx_cmd_id_ = BuildCmdId(msg->is_fire, msg->tracing);
        }

        //定时发送
        void LcSerialTestNode::sendLoop()
        {
            if(!running_) return; // 节点析构了才退出

            // 1. 如果串口没打开，尝试重连
            if (!serial_driver_->port() || !serial_driver_->port()->is_open())
            {
                // 注意：openPort 内部自带锁，不要在这里加 serial_mutex_，否则死锁
                openPort(); 
            }

            // 2. 加锁拷贝数据
            TxData local_data;
            uint16_t local_cmd_id;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                local_data = tx_data_;
                local_cmd_id = tx_cmd_id_;
            }

            // 3. 打包
            uint8_t tx_buf[TX_FRAME_LEN] = {0};
            uint16_t tx_len=0;
            pack_frame(local_cmd_id, local_data, tx_buf, &tx_len);

            // 4. 发送
            bool send_ok = false;
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                try
                {
                    if(serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open())
                    {
                        std::vector<uint8_t> buf(tx_buf, tx_buf + tx_len);
                        serial_driver_->port()->send(buf);
                        send_ok = true;
                    }
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(get_logger(), "Serial send failed: %s", e.what());
                }
            }
        }

        //接收线程
        void LcSerialTestNode::receiveLoop()
        {
            rx_buffer_.resize(1024);

            while(rclcpp::ok() && running_)
            {
                try
                {
                    size_t n = serial_driver_->port()->receive(rx_buffer_);
                    if(n == 0) continue;

                    //滑动窗口找SOF，处理粘包/断包
                    size_t offset = 0;
                    while(offset + RX_FRAME_LEN <= n)
                    {
                        //不是SOF，滑动一字节
                        if(rx_buffer_[offset] != PROTOCOL_SOF)
                        {
                            ++offset;
                            continue;
                        }

                        RxFrame frame;
                        if(unpack_frame(&rx_buffer_[offset], n - offset, frame))
                        {
                            //保存到成员（短持锁）
                            {
                                std::lock_guard<std::mutex> lock(data_mutex_);
                                rx_frame_ = frame.data;
                            }
                            offset += RX_FRAME_LEN;

                            //单位换算(度 -> 弧度)
                            float yaw_rad = static_cast<float>(frame.data.yaw * (M_PI / 180.0));
                            float pitch_rad = static_cast<float>(frame.data.pitch * (M_PI / 180.0));
                            
                            //joint_state(TF 方向旋转)
                            float tf_pitch = pitch_rad;
                            float tf_yaw = yaw_rad;
                            if(recv_tf_reverse_pitch_) tf_pitch = -tf_pitch;
                            if(recv_tf_reverse_yaw_) tf_yaw = -tf_yaw;

                            sensor_msgs::msg::JointState joint_state;
                            joint_state.header.stamp = now();
                            joint_state.name.push_back("gimbal_pitch_joint");
                            joint_state.name.push_back("gimbal_yaw_joint");
                            joint_state.position.push_back(tf_pitch);
                            joint_state.position.push_back(tf_yaw);
                            joint_state_pub_->publish(joint_state);

                            //gimbal_feed 方向反转
                            if(recv_reverse_pitch_) pitch_rad = -pitch_rad;
                            if(recv_reverse_yaw_) yaw_rad = -yaw_rad;

                            auto_aim_interfaces::msg::GimbalFeed feed;
                            feed.pitch = pitch_rad;
                            feed.yaw = yaw_rad;
                            gimbal_feed_pub_->publish(feed);
                        }
                        else
                        {
                            ++offset; //解包失败，滑动一字节
                        }
                    }
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(get_logger(), "Serial receive failed: %s", e.what());
                    // 加锁重连
                    closePort();
                    openPort();
                    std::this_thread::sleep_for(200ms); //避免频繁报错
                }
            }
        }

}

//main
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lc_serial::LcSerialTestNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

