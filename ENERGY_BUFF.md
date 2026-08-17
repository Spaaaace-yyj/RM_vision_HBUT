# 能量机关自动瞄准接入说明

## 一、背景

本工程原有装甲板自动瞄准链路，本次接入能量机关大符与小符的检测和打击功能。

算法部分融合浙江大学开源代码 HWauto_buff2026，将其 5 个算法模块 vendor 进工程，并新增一个 ROS2 节点完成全链路，最终以本工程装甲板的目标消息格式输出，直接供既有 gimbal_controller 使用，不需要改动云台侧代码。

由于本机无 NVIDIA GPU，TensorRT 不可用，且 OpenCV 4.5.4 的 cv::dnn 无法加载该模型的 attention 节点，推理部分改用 onnxruntime CPU 重新实现，算法其余部分全部复用浙大代码。

## 二、本次改动内容

### 1. 新增 buff_detector 包

路径：`src/rm_auto_aim/buff_detector/`

- `third_party/buff_algo/`：浙大算法库源码，包括检测输出解码、筛选、PnP 位姿解算、跟踪、预测五个模块。只排除了依赖 TensorRT 的 `detector/buff_detector.cpp`，其余全部编译进工程。
- `third_party/onnxruntime/`：onnxruntime CPU 推理库及头文件，随仓库分发，无系统安装。
- `src/onnx_buff_detector.cpp`：自写的推理类，接口与浙大 `BuffDetector` 保持一致，letterbox、输出解码等仍复用浙大代码。
- `src/buff_detector_node.cpp`：ROS2 组件节点，单节点完成 订阅图像 → 检测 → 筛选 → PnP → tf 变换 → 跟踪 → 预测 → 发布目标 的全链路。
- `model/`：buff.onnx 模型与 buff.mp4 测试视频。
- 节点参数：
  - `buff_mode`：1=小符，2=大符，可在 launch 中切换
  - `buff_color`：0=打蓝符，1=打红符
  - `model_path`：onnx 模型路径
  - `target_frame`：目标坐标系，离线调试用相机系，实车用 odom

### 2. 目标消息复用

输出直接复用装甲板的 `Target.msg`，gimbal_controller 无需任何改动：

- position：预测打击点
- velocity：打击点切向线速度
- yaw / v_yaw：扇叶 roll 角与角速度
- armors_num = 1，radius_1 = 0，type = "buff"

打击提前量统一由 gimbal_controller 的弹道解算和一级直线外推完成，buff 节点只输出当前时刻的打击点，避免两层提前量叠加。

### 3. launch 重写

路径：`src/bringup/launch/energy_launch.py`

video_pub 与 buff_detector 放进同一个组件容器，图像进程内零拷贝传递。buff_mode、buff_color、video_path 均可在 launch 命令行覆盖。原文件指向不存在的 energy_detector 包，已整体重写。

### 4. 工程修复

- `video_pub`：cv_bridge 只传递 OpenCV core/imgproc/imgcodecs，`cv::VideoCapture` 需要的 videoio 必须显式链接，CMakeLists 中补上。
- `lc_serial`：删除 package.xml 中不存在的 `buff_interfaces` 依赖。
- `video_pub/config/camera_info_buff.yaml`：新增 1280x768 假内参，供离线测试使用。

### 5. 离线测试工具

- `src/rm_auto_aim/buff_detector/src/test_buff_detector.cpp`：检测器回归测试，读视频逐帧检测并输出带关键点标注的视频。
- `src/rm_auto_aim/buff_detector/src/test_buff_pipeline.cpp`：算法全链路回归测试，不依赖 ROS2，输出跟踪状态和打击点投影。
- `src/rm_auto_aim/buff_detector/scripts/view_debug.py`：订阅 `/buff/debug_image` 的轻量图像查看器，按 q 退出。

## 三、目前进度

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| 0 | 模型可用性验证：onnxruntime CPU 推理，buff.mp4 全部帧检出扇叶，约 24 FPS | 完成 |
| 1 | ROS2 全链路节点编写，离线验证进入 TRACKING，输出位置、角速度合理 | 完成 |
| 2a | energy_launch.py 重写，一条命令启动全链路 | 完成 |
| 2c | 端到端验证：buff 节点输出接入 gimbal_controller，配合假云台反馈，输出角度正确，开火判定逻辑正常 | 完成 |

离线验证结果：测试视频运行约 20 秒进入 TRACKING 状态，大符模式下输出距离约 3 米，扇叶角速度符合正弦变速运动，预测打击点稳定落在扇叶上。

期间修复的一个关键问题：video_pub 发布 rgb8 编码图像，检测器内部又会做一次 RGB 交换，两处叠加导致网络实际收到 BGR 图像、颜色类别全部判错，蓝符被当成红符筛掉，状态机一直无法进入跟踪。修复为节点统一按 BGR 解码。

## 四、上实车还需要做什么

1. **launch 实车化**：把 energy_launch.py 中的 video_pub 换成真实相机节点，加入 lc_serial、gimbal_controller 与 tf 广播，target_frame 改为 odom，buff_color 改为 1。当前 launch 只包含离线链路，实车链路需要新写一个 launch 或改造现有文件。串口依赖（ros-humble-serial-driver、asio-cmake-module）2026-08-15 已装好，lc_serial 可正常编译。
2. **launch 实车化**：把 energy_launch.py 中的 video_pub 换成真实相机节点，加入 lc_serial、gimbal_controller 与 tf 广播，target_frame 改为 odom，buff_color 改为 1。当前 launch 只包含离线链路，实车链路需要新写一个 launch 或改造现有文件。
3. **相机标定**：用真实标定结果替换假内参 camera_info_buff.yaml，确认内参格式和发布方式。
4. **真机环境测试**：模型按浙大场地与相机拍摄条件训练，实车光线、距离、视角不同，需要先用真实场地视频验证检出率，必要时调低置信度阈值或重新训练。
5. **算法参数标定**：大符 RANSAC 的角速度范围等参数按我们场地实际转速重新标定；小符固定 60 度/秒的假设需与官方规则和场地实测核对。
6. **无弹联调**：不开火跑通全链路，验证云台角度收敛与开火区间判定，确认弹道参数和 shoot_speed 正确后再实弹。
7. **性能**：当前 CPU 推理约 24 FPS，若实车计算平台带 NVIDIA GPU，可把 onnxruntime 换回浙大原版 TensorRT 推理。

## 五、后续优化（2026-08-15）

1. **tf 查询时间戳**：原来用节点当前时间查相机位姿，云台转动时相机位姿是
   当前时刻的，和图像采集时刻对不上；改为用图像时间戳查 tf，回放无时间戳才兜底。
2. **丢失输出清零**：LOST 时原来会把 (0, -R, 0) 这种无意义点发给云台；
   现在 TRACKING/TEMP_LOST 才填位置，丢失时 armors_num 置 0 通知云台，
   和装甲板链路的丢目标语义一致。
3. **RANSAC 降频**：大符 RANSAC 原来每帧跑 200 次迭代拟合正弦参数，
   参数变化很慢没必要；新增 ransac_fit_interval 参数（默认 5 帧拟合一次），
   降低 CPU 占用，收敛速度不受影响（离线 695 帧回归测试通过）。

## 六、目前用法（2026-08-15）

### 离线调试（本机）

一条命令启动全链路（video_pub 回放 buff.mp4）：

    ros2 launch bringup energy_launch.py

launch 参数可覆盖：

    ros2 launch bringup energy_launch.py buff_mode:=2 buff_color:=0 video_path:=buff.mp4

- buff_mode：1=小符（固定转速 KF），2=大符（RANSAC 正弦拟合）
- buff_color：0=打蓝符（离线视频是蓝符），1=打红符（实车蓝队用）
- video_path：回放视频文件名，放 video_pub/video/ 下

看调试图像（节点 debug_image:=true 时发布 /buff/debug_image，按 q 退出）：

    /usr/bin/python3 src/rm_auto_aim/buff_detector/scripts/view_debug.py

算法回归测试（不依赖 ROS2，大符模式跑完整视频）：

    cd src/rm_auto_aim/buff_detector
    ../../install/buff_detector/lib/buff_detector/test_buff_pipeline model/buff.mp4 2 0 /tmp/out.avi

### buff_detector_node 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| model_path | buff.onnx | onnx 模型路径，launch 里用 share 目录绝对路径 |
| confidence_threshold | 0.5 | 检测置信度，真机检出率低可调低 |
| nms_threshold | 0.4 | NMS 阈值 |
| buff_mode | 2 | 1=小符 2=大符 |
| buff_color | 1 | 0=打蓝符 1=打红符 |
| target_frame | odom | 离线调试用 camera_optical_frame 跳过 tf，实车 odom |
| camera_frame | camera_optical_frame | 相机坐标系 |
| debug_image | false | 发布 /buff/debug_image |
| subscribe_compressed | false | 订阅压缩图像 |
| switch_buff_angle | 45.8 | 扇叶 roll 跳变判定阈值（度） |
| kf_q_pos / kf_q_vel / kf_r_meas | 0.1 / 1.0 / 0.1 | 小符 roll/yaw 卡尔曼噪声 |
| small/big_max_temp_lost_frames | 20 | 临时丢失帧数上限 |
| big_max_converging_frames | 300 | 大符收敛兜底帧数 |
| ransac_max_iterations | 200 | RANSAC 迭代次数 |
| ransac_min/max_omega | 1.884 / 2.0 | 大符角速度搜索范围 |
| ransac_min/max_amplitude | 0.78 / 1.045 | 正弦幅值范围 |
| ransac_min_inliers | 100 | 达到该内点数即进 TRACKING |
| ransac_fit_interval | 5 | 每隔几帧拟合一次 RANSAC，2026-08-15 新增 |

### 实车改动清单

1. launch 里 video_pub 换成真实相机节点（mv_camera / dji_Action4），内参换真实标定
2. target_frame 改 odom，配好 tf 树 odom←gimbal←camera
3. buff_color 改 1（蓝队打红符）
4. 加入 lc_serial、gimbal_controller（组装方式参考 armor_launch.py）
5. 无弹联调：不开火验证云台角度收敛与开火区间，确认 shoot_speed 后再实弹
