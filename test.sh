source /opt/ros/humble/setup.bash
source install/setup.bash
# ros2 launch bringup armor_launch.py use_serial:=True
rqt
# ros2 launch bringup armor_launch.py use_serial:=False
# ros2 launch bringup trt_armor_launch.py  use_serial:=True
imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)