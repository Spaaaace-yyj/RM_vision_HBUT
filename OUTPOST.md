# 前哨站自动瞄准说明

## 一、背景

2026 赛季规则里前哨站改成了旋转模块上高低排布的三块小装甲板：

- 三块板绕竖直轴 120° 均匀分布，随中部装甲整体匀速旋转，转速 0.8π rad/s（约 144°/s），方向当局固定随机
- 三块板离地高度不同，按规则书范围约 1100~1446mm，本工程默认按 1.10 / 1.27 / 1.45 m 处理
- 打哪一块都扣同一个前哨站的血（1500 血），装甲板中心有 10mm×10mm 暴击区，命中伤害 150%
- 中部装甲比赛开始即转，前哨站被击毁、对方基地护甲展开或比赛 3 分钟后停止

## 二、方案

装甲板检测链路本来就支持 outpost 类别（分类器 label 里有 outpost，小装甲板放行），
前哨站代码只做"预测旋转轨迹 + 选正对的板"这件事。

三块板共轴共半径、只有高度不同，所以用**一个共享的圆周运动 EKF** 建模
（参照 armor_tracker 的 ExtendedKalmanFilter 写法）：

- 状态 6 维：`[轴心x, 轴心y, 相位, 板0高, 板1高, 板2高]`，半径 r 和转速是机械固定值用参数，不进状态
- 观测：任意一块板的位置，按高度归类到对应板索引后顺序更新（带位置门控，防误检污染）
- 相位直接由"被观测板正对相机"反推初始化，窗口间用转速外推，观测窗口内拉回
- 板转到背面时 EKF 继续外推，定时器 20Hz 持续发布目标，云台提前转到下一块正对板的位置

转速处理（规则书给定匀速，方向当局固定随机）：

- 方向：第一观测窗口内相位差分自动测定（rotate_direction=0），也可手动指定 ±1
- 停转：3 分钟后模块减速停止，检测到相位不再变化就把转速置 0，输出速度跟着归零
- 半径 r 如果和实车有出入，参数 r_initial 直接改

## 三、新增内容

### 1. 新包 outpost_detector

路径：`src/rm_auto_aim/outpost_detector/`

- `src/outpost_target_node.cpp`：组件节点，订阅检测结果 → EKF 跟踪旋转轨迹 → 发布 Target
- `scripts/fake_outpost.py`：合成测试，模拟三块高低板旋转，不依赖相机和模型

### 2. launch

路径：`src/bringup/launch/outpost_launch.py`

video_pub + armor_detector + outpost_target_node 一个容器跑，离线调试用。
实车一条命令：`ros2 launch bringup outpost_real_launch.py`（mv_camera + armor_detector +
outpost_target_node + gimbal_controller + lc_serial + tf），
前哨站位置门：`ros2 launch bringup outpost_real_launch.py outpost_x:=3.0 outpost_y:=0.0`。

### 3. 目前用法

离线启动（video_pub 回放 + 检测 + 前哨站节点）：

    ros2 launch bringup outpost_launch.py video_path:=outpost.mp4

launch 参数：video_path（回放视频，放 video_pub/video/ 下）、outpost_x / outpost_y（位置门）。

手动三终端启动（不依赖 launch，方便调试）：

    # 终端 1：视频回放
    ros2 run video_pub video_pub_node --ros-args \
      -p video_path:=outpost.mp4 -p fps:=30 \
      -p camera_info_url:=package://video_pub/config/camera_info_buff.yaml \
      -p use_sensor_data_qos:=True
    # 终端 2：装甲板检测 + 前哨站节点
    ros2 run armor_detector armor_detector_node
    ros2 run outpost_detector outpost_target_node --ros-args \
      -p target_frame:=odom -p camera_frame:=odom

合成数据测试（没有真实视频时验证逻辑）：

    ros2 run outpost_detector outpost_target_node --ros-args \
      -p target_frame:=odom -p camera_frame:=odom
    # 另开终端
    python3 install/outpost_detector/lib/outpost_detector/fake_outpost.py
    # 再开终端看输出
    ros2 topic echo /tracker/target

注意：调试时只跑一个 outpost_target_node，多开实例会互相抢 /tracker/target。

### 4. 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| slot_z | [1.10, 1.27, 1.45] | 三块板离地高度，实车按场地实测改 |
| z_tol | 0.08 | 高度带半宽，用于把观测归到板索引 |
| r_initial | 0.28 | 旋转半径，模块机械尺寸 |
| rotate_direction | 0 | 旋转方向，0=第一窗口自动测，1/-1=手动指定 |
| gate_dist | 1.0 | 观测门控，预测位置离观测太远就拒绝 |
| temp_lost_time | 1.5 | 观测中断这么久进 TEMP_LOST，比旋转周期长就不会掉状态 |
| lost_time | 5.0 | 观测中断这么久回 LOST |
| tracking_thres | 5 | 连续观测这么多帧进 TRACKING |
| outpost_x/y/radius | 0/0/0 | 位置门，半径 0 表示关闭 |

## 四、注意事项

1. **速度提前量**：输出速度 = 转速 × 半径的切向速度（实测 0.70 m/s 与理论 ωR 吻合），
   云台用它做直线外推；停转后速度自动归零。
2. **为什么输出会跟着换板**：每帧选最接近正对相机的板（带滞回防边界抖动），
   三块板轮流正对，输出位置自然在三个高度间切换，云台不用等板转回来。
3. **开火时机**：云台只在目标消息到达时判断开火区间，板正对时消息里的位置
   就是它的预测位置，窗口内才会打进开火区。
4. **跟丢语义**：armors_num=0 表示丢失（云台按丢目标处理），和装甲板链路一致；
   不要发 armors_num=1 加零位置，会把云台引到原点。
5. **测试教训**：多开几个节点实例会互相抢 /tracker/target，看起来就像状态漂移，
   调试时只跑一个节点。
6. **实车待办**：slot_z、r_initial 按场地实测改；outpost_x/y 位置门按需开；
   验证 tf 树 odom←gimbal←camera；用真实场地视频验证 outpost 检出率；
   赛前确认旋转方向（自动测定即可，不放心就手动指定）。
