#!/usr/bin/env python3
# 在同一段视频上对比传统模式与神经网络模式的检出情况
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from auto_aim_interfaces.msg import Armors


class Counter(Node):
    def __init__(self):
        super().__init__('ab_counter')
        self.create_subscription(
            Armors, '/detector/armors', self.cb, qos_profile_sensor_data)
        self.msgs = 0
        self.nonempty = 0
        self.total = 0
        self.numbers = {}

    def cb(self, msg):
        self.msgs += 1
        if msg.armors:
            self.nonempty += 1
            self.total += len(msg.armors)
            for a in msg.armors:
                key = f'{a.number}/{a.type}'
                self.numbers[key] = self.numbers.get(key, 0) + 1

    def reset(self):
        self.msgs = self.nonempty = self.total = 0
        self.numbers = {}


def run(node, seconds):
    node.reset()
    end = node.get_clock().now().nanoseconds + seconds * 1e9
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return node.msgs, node.nonempty, node.total, dict(node.numbers)


def main():
    rclpy.init()
    node = Counter()
    # 等话题连上
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)

    print('>>> 预热 5s（传统模式）')
    run(node, 5)

    print('>>> 传统模式 20s')
    m1, ne1, t1, n1 = run(node, 20)
    print(f'    收到 {m1} 帧, 有检出 {ne1} 帧, 装甲板 {t1} 个, 平均 {t1 / max(m1, 1):.2f} 个/帧')
    print(f'    类别统计: {n1}')

    subprocess.run(
        ['ros2', 'param', 'set', '/armor_detector', 'detector_mode', 'neural'],
        check=False, capture_output=True)
    print('>>> 神经网络模式 20s')
    m2, ne2, t2, n2 = run(node, 20)
    print(f'    收到 {m2} 帧, 有检出 {ne2} 帧, 装甲板 {t2} 个, 平均 {t2 / max(m2, 1):.2f} 个/帧')
    print(f'    类别统计: {n2}')

    print('\n================ 汇总 ================')
    print(f'传统识别 : 有检出帧 {ne1}/{m1} ({100.0 * ne1 / max(m1, 1):.1f}%), 装甲板 {t1}')
    print(f'神经网络 : 有检出帧 {ne2}/{m2} ({100.0 * ne2 / max(m2, 1):.1f}%), 装甲板 {t2}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
