#include<iostream>
#include<rclcpp/rclcpp.hpp>
#include<buff_interfaces/msg/rune.hpp>
#include<buff_interfaces/msg/rune_info.hpp>
#include<buff_interfaces/msg/debug_blade_array.hpp>
#include<buff_interfaces/msg/debug_blade.hpp>

using namespace rclcpp;
using namespace std;

class BuffInterfaceTestNode : public Node{
    public:
        BuffInterfaceTestNode() : Node("test_node"){
            RCLCPP_INFO(this->get_logger(), "Buff interface test node Started!");

            Rune_subscription_ = this->create_subscription<buff_interfaces::msg::Rune>("tracker/rune", 10, std::bind(&BuffInterfaceTestNode::Rune_callback, this, std::placeholders::_1));
            RuneInfo_subscription_ = this->create_subscription<buff_interfaces::msg::RuneInfo>("tracker/rune_info", 10, std::bind(&BuffInterfaceTestNode::RuneInfo_callback, this, std::placeholders::_1));
            debug_bladeArry_subscription_ = this->create_subscription<buff_interfaces::msg::DebugBladeArray>("/debug/blade_array", 10, std::bind(&BuffInterfaceTestNode::debug_bladeArry_callback, this, std::placeholders::_1));
        }
        void Rune_callback(const buff_interfaces::msg::Rune msg){
            cout << "Rune_callback" << endl;
            cout << msg.tracking << endl;
        }
        void RuneInfo_callback(const buff_interfaces::msg::RuneInfo msg){
            cout << "RuneInfo_callback" << endl;
        }

        void debug_bladeArry_callback(const buff_interfaces::msg::DebugBladeArray msg){
            cout << "debug_bladeArry_callback" << endl;
            cout << msg.blades.size() << endl;
            //cout << msg.blades[0].pose.position.x << endl;
        }
    private:
        rclcpp::Subscription<buff_interfaces::msg::Rune>::SharedPtr Rune_subscription_;
        rclcpp::Subscription<buff_interfaces::msg::RuneInfo>::SharedPtr RuneInfo_subscription_;
        rclcpp::Subscription<buff_interfaces::msg::DebugBladeArray>::SharedPtr debug_bladeArry_subscription_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = make_shared<BuffInterfaceTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
