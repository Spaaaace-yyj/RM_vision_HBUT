#!/usr/bin/python3
# 查看 /buff/debug_image 的轻量查看器，按 q 退出。
# 显示绿色关键点和黄色预测打击点。
# 用法: python3 view_debug.py [话题名]
# 注意：必须用 /usr/bin/python3，PATH 里 platformio 的 python 没有 cv2。
import sys

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/buff/debug_image'

rclpy.init()
node = rclpy.create_node('debug_viewer')
img = {'frame': None}


def cb(msg):
    img['frame'] = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


node.create_subscription(
    Image, TOPIC, cb,
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

print(f'viewing {TOPIC}, press q to quit')
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    if img['frame'] is not None:
        cv2.imshow('buff debug', img['frame'])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
rclpy.shutdown()
