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
           
            // 初始化为当前时间，防止一启动就被判定超时
            last_cmd_vel_time_ = std::chrono::steady_clock::now();
            last_gimbal_control_time_ = std::chrono::steady_clock::now();

            //参数
            device_name_           = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
            send_reverse_pitch_    = declare_parameter<bool>("send_reverse_pitch", true);
            send_reverse_yaw_      = declare_parameter<bool>("send_reverse_yaw", false);
            recv_reverse_pitch_    = declare_parameter<bool>("recv_reverse_pitch", true);
            recv_reverse_yaw_      = declare_parameter<bool>("recv_reverse_yaw", true);
            recv_tf_reverse_pitch_ = declare_parameter<bool>("recv_tf_reverse_pitch", true);
            recv_tf_reverse_yaw_   = declare_parameter<bool>("recv_tf_reverse_yaw", false);
            // 新增：将三个控制状态改为参数，并读取覆盖初始值
            // 注意：ROS 2 参数通常使用 int，读取后强转为 uint16_t
            chassis_cmd_ = static_cast<uint16_t>(declare_parameter<int>(
                            "chassis_cmd", static_cast<int>(ChassisCmd::NO_FOLLOWING)));

            gimbal_cmd_ = static_cast<uint16_t>(declare_parameter<int>(
                            "gimbal_cmd", static_cast<int>(GimbalCmd::OFF)));

            shoot_cmd_ = static_cast<uint16_t>(declare_parameter<int>(
                            "shoot_cmd", static_cast<int>(ShootCmd::OFF)));

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
            
                
            receive_thread_ = std::thread(&LcSerialTestNode::receiveLoop, this);
            

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

        //订阅回调
        void LcSerialTestNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            tx_data_.forward = static_cast<float>(msg->linear.x);
            tx_data_.turn    = static_cast<float>(msg->linear.y);
            tx_data_.rotate  = static_cast<float>(msg->angular.z);

            // 记录最后收到指令的时间
            last_cmd_vel_time_ = std::chrono::steady_clock::now();
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
            // 新增：只要自瞄发了数据，就自动让云台使能
            gimbal_cmd_ = static_cast<uint16_t>(GimbalCmd::ENABLE);
            tx_data_.yaw = yaw_deg;
            tx_data_.pitch = pitch_deg;
            if (msg->is_fire) {
                shoot_cmd_ = static_cast<uint16_t>(ShootCmd::FIRE);
            } else if (msg->tracing) {
                shoot_cmd_ = static_cast<uint16_t>(ShootCmd::FICTION_ON);
            } else {
                shoot_cmd_ = static_cast<uint16_t>(ShootCmd::OFF);
            }

            //记录最后收到指令的时间
            last_gimbal_control_time_ = std::chrono::steady_clock::now();
        }

        //定时发送
        void LcSerialTestNode::sendLoop()
        {
            if(!running_) return; // 节点析构了才退出

            // 1. 安全地检查串口状态（加锁）
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (!serial_driver_->port() || !serial_driver_->port()->is_open())
                {
                    // 直接退出本次发送，等待接收线程报错重连
                    return; 
                }
            }

                // 检查指令超时 (100ms)
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_vel_time_).count();
                    auto elapsed2 = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_gimbal_control_time_).count();
                    if (elapsed1 > 100) {
                        // 超过 100ms 没收到速度，强制清零
                        tx_data_.forward = 0.0f;
                        tx_data_.turn = 0.0f;
                        tx_data_.rotate = 0.0f;
                        
                    }
                    if (elapsed2 > 100)
                    {
                        gimbal_cmd_ = static_cast<uint16_t>(GimbalCmd::OFF);
                    }
                }

            // 2. 加锁拷贝数据
            TxData local_data;
            uint16_t local_cmd_id = 0;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                local_data = tx_data_;
                // 动态拼装
                
                local_cmd_id |= (chassis_cmd_ << CHASSIS_CMD_SHIFT);
                local_cmd_id |= (gimbal_cmd_  << GIMBAL_CMD_SHIFT);
                local_cmd_id |= (shoot_cmd_   << SHOOT_CMD_SHIFT);

            }

            // 3. 打包
            uint8_t tx_buf[TX_FRAME_LEN] = {0};
            uint16_t tx_len=0;
            pack_frame(local_cmd_id, local_data, tx_buf, &tx_len);

            // 4. 发送
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                try
                {
                    if(serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open())
                    {
                        std::vector<uint8_t> buf(tx_buf, tx_buf + tx_len);
                        serial_driver_->port()->send(buf);
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
                    if (!running_) break; // 如果节点正在退出，直接结束线程，不要重连！
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

