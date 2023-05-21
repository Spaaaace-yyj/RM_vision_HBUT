source /opt/ros/galactic/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch bringup armor_launch.py use_serial:=True
# ros2 launch bringup trt_armor_launch.py  use_serial:=True
