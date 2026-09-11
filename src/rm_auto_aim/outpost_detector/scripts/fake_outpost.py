#!/usr/bin/python3
# 前哨站合成测试：假装是装甲板检测节点，模拟三块高低排布的小装甲板
# 绕竖直轴匀速旋转，只有正对"相机"的板才发出去，验证 outpost_target_node 的
# 槽位锁定和切换逻辑。用法：
#   ros2 run outpost_detector outpost_target_node --ros-args \
#     -p target_frame:=odom -p camera_frame:=odom
#   然后另开终端跑本脚本，再 echo /tracker/target 观察输出

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time

from auto_aim_interfaces.msg import Armor, Armors

# 三块板的离地高度，和规则书一致：1100~1446mm
PLATE_Z = [1.10, 1.27, 1.45]
# 旋转半径，和老规则一样按 0.2765m 假
RADIUS = 0.2765
# 模块中心在 odom 系的位置
CENTER = (3.0, 0.0)
# 转速 0.8π rad/s，方向按逆时针
OMEGA = 0.8 * math.pi
# 初始方位角，低中高三块板 120° 分布
AZIMUTH0 = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]
# 正对窗口半宽，约 14°，窗口内才"检测"得到
FACING_HALF = 0.24


class FakeOutpost(Node):
    def __init__(self):
        super().__init__('fake_outpost')
        self.pub = self.create_publisher(Armors, '/detector/armors', qos_profile_sensor_data)
        self.timer = self.create_timer(0.02, self.tick)  # 50Hz
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        random.seed(42)

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        msg = Armors()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'odom'

        for k in range(3):
            # 板 k 的方位角，正对"相机"（x 轴正方向）时才能检测到
            az = AZIMUTH0[k] + OMEGA * t
            az = (az + math.pi) % (2 * math.pi) - math.pi
            if abs(az) > FACING_HALF:
                continue

            armor = Armor()
            armor.number = 'outpost'
            armor.type = 'small'
            armor.pose.position.x = CENTER[0] + RADIUS * math.cos(az) + random.gauss(0, 0.01)
            armor.pose.position.y = CENTER[1] + RADIUS * math.sin(az) + random.gauss(0, 0.01)
            armor.pose.position.z = PLATE_Z[k] + random.gauss(0, 0.005)
            armor.pose.orientation.w = 1.0
            msg.armors.append(armor)

        if msg.armors:
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeOutpost()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
