# 能量机关自动瞄准说明

## 一、背景

本工程原有装甲板自动瞄准链路，能量机关（大符+小符）检测与打击基于浙江大学开源代码
HWauto_buff2026 融合实现：算法模块 vendor 进 `src/rm_auto_aim/buff_detector/third_party/buff_algo/`，
推理改用 onnxruntime CPU（本机无 NVIDIA GPU，TensorRT 不可用），
新增一个 ROS2 节点完成 检测→筛选→PnP→tf→跟踪→预测→发布目标 全链路，
输出复用装甲板的 `Target.msg`，云台侧零改动。

## 二、目前代码状态（2026-08-15）

### 1. 包结构

```
src/rm_auto_aim/buff_detector/
├── third_party/buff_algo/          # 浙大算法库（筛选/PnP/跟踪/预测，只去掉 TensorRT 检测）
├── third_party/onnxruntime/        # onnxruntime CPU 推理库（随仓库分发）
├── model/
│   ├── buff.onnx                   # 浙大模型（默认，640x384 输入，9 关键点 4 类）
│   └── buff.mp4                    # 浙大测试视频（gitignore，不提交）
├── src/
│   ├── onnx_buff_detector.cpp      # 推理类，支持 zju/szu 两种模型格式
│   ├── buff_detector_node.cpp      # ROS2 节点，全链路
│   ├── test_buff_detector.cpp      # 检测器回归测试
│   └── test_buff_pipeline.cpp      # 算法全链路回归测试
└── scripts/view_debug.py           # 调试图像查看器
```

### 2. 支持两种检测模型（detector_format 参数切换）

| 格式 | 模型 | 输入 | CPU 实测 | 类别 | 说明 |
| --- | --- | --- | --- | --- | --- |
| zju（默认） | buff.onnx | 640x384 | 约 38ms（26FPS） | 4 类（蓝/红×未激活/激活） | 浙大原版，全链路验证过 |
| szu | 深大 model-0624.onnx | 640x480 | 约 19-24ms（42-51FPS） | 3 类（未激活/小符激活/大符激活） | 深大开源，快一倍，需实车验证 |

szu 模式会自动做：
- 关键点格式转换：深大 5 点（R 字四角 + R 中心）按象限排序后取四边中点，
  转成浙大的 上/左/下/右 方向点
- 颜色判定：深大模型没有颜色类别，用检测框 ROI 内红蓝像素统计补上
- 类别映射：深大"未激活"→浙大 INACTIVATE（要打的），激活态→ACTIVATE（被筛掉）

**szu 模型是深大自己场地数据训练的（14k 张，分区赛到国赛验证），
在浙大场地视频（buff.mp4）上能检出但关键点不稳定，离线测试进不了 TRACKING。
真实场地效果必须实车验证。**

### 3. 近期改动记录

2026-08-15 一批改动（已提交）：
- **tf 查询时间戳**：用图像采集时刻查 tf（原来用 now()，云台转动时会偏）
- **丢失输出清零**：LOST 时 armors_num 置 0，不再发 (0,-R,0) 垃圾点
- **RANSAC 降频**：大符 RANSAC 每 5 帧拟合一次（参数 ransac_fit_interval），
  原每帧 200 次迭代，省 CPU，收敛不受影响（695 帧回归通过）
- **深大模型支持**：detector_format 参数切换（见上表）
- **PnP 异常位姿过滤**：非有限值（NaN）位姿直接丢弃，防止污染跟踪器
- **buff_radius 参数化**：R 中心到打击点的距离，浙大写死 0.7m，
  现改为参数（默认 0.7，行为不变），实车按机械尺寸标定

**已回退/放弃的尝试（记录防止重复踩坑）**：
- R 中心关键点修正 PnP 平移：实测把黄圈拉飞，已删除
- onnxruntime 多线程/图优化配置：A/B 实测比默认慢（47ms vs 38ms），保持默认
- 打击点改为 R 中心（靶心）：R 字只是标志，不是打击目标，已恢复浙大原逻辑

## 三、怎么启用

### 1. 离线调试（本机）

```bash
# 一条命令全链路（video_pub 回放 buff.mp4 + 检测 + 跟踪 + 发布目标）
ros2 launch bringup energy_launch.py

# 常用参数覆盖
ros2 launch bringup energy_launch.py buff_mode:=2 buff_color:=0
ros2 launch bringup energy_launch.py buff_mode:=1          # 小符

# 看调试图像（黄圈=打击点，绿点=关键点），按 q 退出
/usr/bin/python3 src/rm_auto_aim/buff_detector/scripts/view_debug.py
```

**离线参数默认值**：buff_mode=2（大符）、buff_color=0（打蓝符，
因为 buff.mp4 里是蓝符）、video_path=buff.mp4。

**离线换测试视频**：把视频文件放到 `src/video_pub/video/` 目录下
（mp4/avi 已被 gitignore，本地放不会提交），然后：
`ros2 launch bringup energy_launch.py video_path:=你自己的视频.mp4`

### 2. 算法回归测试（不依赖 ROS2，改代码后必跑）

```bash
cd src/rm_auto_aim/buff_detector
# 浙大模型全链路（大符）：正常应 TRACKING in ~393 frames
../../install/buff_detector/lib/buff_detector/test_buff_pipeline model/buff.mp4 2 0 /tmp/out.avi

# 深大模型（szu 模式，第 6 个参数是模型路径）
../../install/buff_detector/lib/buff_detector/test_buff_pipeline model/buff.mp4 2 0 /tmp/out.avi szu /path/to/model-0624.onnx

# 检测器单独测试（画关键点）
../../install/buff_detector/lib/buff_detector/test_buff_detector model/buff.mp4
```

### 3. 实车（一条命令）

```bash
# 默认大符 + 打红符（buff_color=1，蓝队打红符）
ros2 launch bringup energy_real_launch.py
# 切换大小符
ros2 launch bringup energy_real_launch.py buff_mode:=1
# 切换颜色（红方队伍打蓝符时用 0）
ros2 launch bringup energy_real_launch.py buff_color:=0
```

**实车和离线默认颜色不一样**：实车默认 buff_color=1（打红），
离线默认 buff_color=0（打蓝，视频是蓝符），别搞混。

energy_real_launch.py 组装了：mv_camera + buff_detector（同容器零拷贝）+
gimbal_controller + lc_serial + robot_state_publisher（tf）。
前置条件：
1. 相机标定内参在 params.yaml 的 /mv_camera 里（camera_info_url 指向真实标定文件）
2. tf 树 odom←gimbal←camera 完整（URDF 已带，云台角度来自电控反馈）
3. 串口设备名 /dev/ttyACM0 和电控对好（params.yaml 的 /lc_serial_driver）
4. 哨兵用二进制协议的话把 serial_node 换成 lc_rm_serial 的节点
5. 无弹联调：不开火验证云台角度收敛与开火区间，确认 shoot_speed 后再实弹

## 四、参数表

### buff_detector_node

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| model_path | buff.onnx | 模型路径，launch 里用 share 目录绝对路径 |
| detector_format | zju | 模型格式：zju=浙大原版，szu=深大 5 点模型（快一倍） |
| confidence_threshold | 0.5 | 检测置信度，真机检出率低可调低 |
| nms_threshold | 0.4 | NMS 阈值 |
| buff_mode | 2 | 1=小符（固定转速 KF） 2=大符（RANSAC 正弦拟合） |
| buff_color | 1 | 0=打蓝符 1=打红符（蓝队打红符） |
| buff_radius | 0.7 | R 中心到打击点的距离（米），实车按机械尺寸标定 |
| target_frame | odom | 离线调试用 camera_optical_frame 跳过 tf，实车 odom |
| camera_frame | camera_optical_frame | 相机坐标系 |
| debug_image | false | 发布 /buff/debug_image |
| subscribe_compressed | false | 订阅压缩图像 |
| switch_buff_angle | 45.8 | 扇叶 roll 跳变判定阈值（度） |
| kf_q_pos / kf_q_vel / kf_r_meas | 0.1 / 1.0 / 0.1 | 小符 roll/yaw 卡尔曼噪声 |
| small/big_max_temp_lost_frames | 20 | 临时丢失帧数上限 |
| big_max_converging_frames | 300 | 大符收敛兜底帧数 |
| ransac_max_iterations | 200 | RANSAC 迭代次数 |
| ransac_min/max_omega | 1.884 / 2.0 | 大符角速度搜索范围（rad/s） |
| ransac_min/max_amplitude | 0.78 / 1.045 | 正弦幅值范围 |
| ransac_min_inliers | 100 | 达到该内点数即进 TRACKING |
| ransac_fit_interval | 5 | 每隔几帧拟合一次 RANSAC（性能优化） |

### launch（energy_launch.py）

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| buff_mode | 2 | 1=小符 2=大符 |
| buff_color | 0 | 离线视频是蓝符用 0，实车蓝队打红符用 1 |
| video_path | buff.mp4 | 回放视频文件名（放 video_pub/video/ 下） |

## 五、后续怎么调整

### 1. 黄圈（打击点）位置标定（优先级最高）

黄圈偏的常见原因和对应调整：
- **固定方向偏**：`buff_radius` 和实车实际距离不符。
  在场地量一下 R 中心到打击点的真实距离，改参数：
  `ros2 param set /buff_detector_node buff_radius 0.65`（试到黄圈贴靶）
- **转动中偏**：roll 角速度预测误差（RANSAC 拟合初期正常，稳定后应收敛），
  可调 ransac_min_inliers / ransac_max_iterations 让拟合更快更稳
- **Pitch 偏高/偏低**：检查相机安装角度和 tf 树

### 2. 帧率提升（按顺序试）

1. **实车试 szu 模型**（最快路径）：`detector_format:=szu` + model_path 指向深大 onnx。
   帧率从 26FPS 提到 50FPS。检测稳就用，不稳切回 zju（一行参数）。
   深大 onnx 从 https://github.com/SZURPVision/RuneDetectionModel 下载
   （model/model-0624.onnx），放到 buff_detector/model/ 下。
2. 实车计算平台如果有 NVIDIA GPU：onnxruntime 换 CUDA EP 或浙大原版 TensorRT 推理
3. 相机分辨率降一档（如果相机支持），letterbox resize 开销会小一些

### 3. 打击策略调整

- 小符/大符切换：buff_mode 参数
- 想打环区（高环）：需要改 predictor 的打击点模型（当前打装甲模块中心位置），
  浙大 predictor 的 roll 外推逻辑保留着，可以在 predict_position 基础上扩展
- 串口/云台侧：shoot_speed、shoot_delay 按实车弹道标定（gimbal_controller 参数）

### 4. 换/加数据集重训

- 浙大模型和深大模型都是各自场地训练的，实车场地不同检出率会变。
- 检出率低：先调 confidence_threshold（0.5 → 0.3 试），再考虑采集本场地数据重训
- 重训需要 YOLO 关键点标注（9 点：八边形顶点 + R 中心），训练脚本不在此仓库

## 六、已知问题

1. **szu 模型在浙大视频上关键点不稳**（场景差异），离线无法验证，必须实车
2. CPU 推理是瓶颈（zju 38ms），onnxruntime 配置优化实测无效
3. video_pub 回放 30fps 是帧率上限，实测帧率看检测耗时
4. debug 图像里黄圈是当前时刻打击点投影，弹道提前量在云台侧，
   黄圈和实际弹着点之间还有弹道修正，无弹联调时以云台输出角度为准
