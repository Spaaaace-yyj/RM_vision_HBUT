#!/bin/bash
# 清掉所有视觉相关进程，保留仿真器。
# 每次重启自瞄前跑一次，避免多套实例抢话题。
# 判据是进程命令行，不匹配自己的 shell，所以直接跑没有副作用。

PAT='ros2 launch bringup|armor_detector_node|armor_tracker_node|gimbal_controller|tracker_overlay_node|mujoco_base_to_odom'

pids=$(ps -eo pid,args | grep -E "$PAT" | grep -v 'clean_vision.sh' | grep -v grep | awk '{print $1}')

if [ -z "$pids" ]; then
    echo "没有残留进程"
else
    echo "清理 $(echo "$pids" | wc -l) 个进程"
    echo "$pids" | xargs -r kill -9 2>/dev/null
    sleep 2
fi

left=$(ps -eo args | grep -E 'armor_detector_node|armor_tracker_node|gimbal_controller' | grep -v grep | wc -l)
echo "剩余视觉节点: $left   （0 表示干净）"
