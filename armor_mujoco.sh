#!/usr/bin/env bash
# 启动 MuJoCo 对应的 ROS 自瞄链路；仿真器本体需另行启动。
set -e

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -f "$ros_setup" ]]; then
    echo "找不到 ROS 环境：$ros_setup（可用 ROS_SETUP 指定 setup.bash）" >&2
    exit 1
fi
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
    echo "工作空间尚未编译，请先在 $workspace_dir 执行 bash build.sh" >&2
    exit 1
fi

source "$ros_setup"
source "$workspace_dir/install/setup.bash"
cd "$workspace_dir"

exec ros2 launch bringup armor_launch.py \
    use_mujoco:=True use_sim_time:=True use_serial:=False \
    "$@"
