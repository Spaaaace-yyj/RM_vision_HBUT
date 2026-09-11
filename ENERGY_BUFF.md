# 能量机关自动瞄准说明

## 一、背景与规则理解

本工程原有装甲板自动瞄准链路，能量机关检测与打击基于浙江大学开源代码
HWauto_buff2026 融合实现，推理用 onnxruntime CPU（本机无 NVIDIA GPU），
新增 buff_detector 节点完成全链路，输出复用装甲板 Target.msg，云台侧零改动。

**2026 规则要点**：
- 能量机关中心是 **R 字标志**（只用于观测定位，**不是打击目标**）
- **5 个发光靶子（装甲模块）绕 R 旋转**，打靶子（靶心 = 装甲模块中心），R 只是参照物
- 浙大 0.7m 的思路：从 R 中心沿半径延伸 0.7m，正好到靶子附近
- 大符是正弦变速旋转（速度 = A·sin(ωt+φ)+C），不是匀速

## 二、链路结构

```
相机/video_pub → buff_detector（检测→筛选→PnP→tf→跟踪→预测）
                → /tracker/target（position=目标打击点 + predictive_point=预测击打点）
                → gimbal_controller（弹道解算+提前量+开火判定）
                → control/gimbal_control → lc_serial → 电控
```

**两个打击点的区别**：

| 点 | 含义 | 用途 |
| --- | --- | --- |
| **绿色十字 = 目标打击点** | 当前时刻靶子位置 | gimbal 跟踪用的瞄准点（position 字段），和浙大原版语义一致 |
| **黄圈 = 预测击打点** | 子弹到达时刻靶子位置 | 离线模拟弹着点；实车验证算得准后可反馈云台修正（predictive_point 字段，云台暂未消费） |

## 三、离线使用（本机）

### 1. 一条命令启动

```bash
ros2 launch bringup energy_launch.py
# 常用参数
ros2 launch bringup energy_launch.py buff_mode:=1        # 切小符
ros2 launch bringup energy_launch.py detector_format:=szu  # 切深大模型
ros2 launch bringup energy_launch.py buff_color:=0       # 打蓝符（视频是蓝符）
```

**离线默认**：大符（buff_mode=2）、打蓝符（buff_color=0，buff.mp4 里是蓝符）、浙大模型（zju）。

### 2. 看调试图像

```bash
/usr/bin/python3 src/rm_auto_aim/buff_detector/scripts/view_debug.py
```

图上：**绿色小点** = 模型关键点，**绿色十字** = 目标打击点，**黄圈** = 预测击打点。

⚠️ **只在 TRACKING 状态才画十字和黄圈**——视频要跑 20 秒左右才进 TRACKING（日志打印 status: TRACKING），别在 CONVERGING 阶段看。

### 3. 模型切换（zju / szu）

| 格式 | 模型 | CPU 实测 | 说明 |
| --- | --- | --- | --- |
| zju（默认） | model/buff.onnx（浙大 640×384） | 约 38ms / 26FPS | 全链路验证过，离线就它 |
| szu | 深大 model-0624.onnx（640×480） | 约 20ms / 50FPS | 快一倍，但**只在它训练过的场地稳**，浙大视频上关键点不稳，需实车验证 |

szu 用法：`detector_format:=szu` 且 `model_path` 指向深大 onnx（如 `/home/twy/RuneDetectionModel/model/model-0624.onnx`）。

### 4. 算法回归测试（改代码后必跑）

```bash
cd src/rm_auto_aim/buff_detector
# 浙大模型全链路（大符），正常应 TRACKING in ~393 frames
../../install/buff_detector/lib/buff_detector/test_buff_pipeline model/buff.mp4 2 0 /tmp/out.avi
# 深大模型
../../install/buff_detector/lib/buff_detector/test_buff_pipeline model/buff.mp4 2 0 /tmp/out.avi szu /path/to/model-0624.onnx
```

### 5. 换测试视频

视频文件放 `src/video_pub/video/` 下（mp4/avi 已被 gitignore），然后：
`ros2 launch bringup energy_launch.py video_path:=你的视频.mp4`

## 四、实车使用

### 1. 兵种协议区分（先搞清楚再上车）

视觉侧为**同一套自瞄工程**（本工程），英雄/步兵与哨兵仅串口协议不同，
通过 `serial_protocol` 参数区分，电控侧接收协议与视觉侧一一对应：

| 兵种 | serial_protocol | 串口节点 | 电控协议 |
| --- | --- | --- | --- |
| 英雄 / 步兵 | `cjson`（默认） | lc_serial | CJSON 文本，115200 |
| 哨兵 | `binary` | lc_rm_serial | 二进制帧（0xA5 头 + CRC16），921600 |

协议不匹配时电控无法解析，云台不动或乱动。上车前确认对应兵种的电控程序，
启动时带对应参数。同一台车换兵种使用时，只改这个参数即可。

### 2. 启动

```bash
# 英雄 / 步兵
ros2 launch bringup energy_real_launch.py
# 哨兵
ros2 launch bringup energy_real_launch.py serial_protocol:=binary
# 小符（大符为默认）
ros2 launch bringup energy_real_launch.py buff_mode:=1
```

energy_real_launch.py 自动组装：mv_camera + buff_detector（零拷贝）+
gimbal_controller + 对应协议串口节点 + robot_state_publisher（tf）。
实车默认参数：大符、打红符（buff_color=1，蓝队打红符）、浙大模型、odom 系跟踪。

### 3. 硬件连接

1. 电控上电（云台供电独立于 USB）
2. 相机 USB 线接电脑（海康相机，驱动随系统分发）
3. 串口线接电脑，确认电控侧接口

接头检查：`ls /dev/ttyACM*`（或 ttyUSB*），记下实际设备名。
串口设备名与 USB 口位置相关，固定插同一个口可保持稳定。

### 4. 启动前参数确认（params.yaml）

| 参数 | 确认内容 |
| --- | --- |
| /mv_camera.camera_info_url | 指向当前相机+镜头的标定文件（学长已标定，确认是该文件） |
| /lc_serial_driver.device_name | 与实际串口设备名一致 |
| /gimbal_controller.shoot_speed | 先按已知弹速填写，后续打靶标定 |

### 5. 通信验证（启动后第一步）

```bash
ros2 topic echo /gimbal_feed
```

- 通过：有数据且数值在变化，说明电控在回传 IMU 角度，串口双向通
- 无数据排查顺序：设备名 → 串口权限（usermod -aG dialout）→ 协议不匹配 → 电控上电状态
- 手转云台，确认 /gimbal_feed 中对应角度跟随变化（核对方向约定）

### 6. 无弹联调（关发射机构/保险后运行）

启动后按以下次序确认，全部通过再进入实弹：

1. `ros2 run tf2_tools view_frames`：odom←gimbal←camera 树完整；
   手动转动云台时 rviz 中相机坐标系跟随
2. `ros2 topic echo /tracker/target`：检测到符后 tracking=true、position 数值合理
3. 云台自动跟踪扇叶；`/debug/controller` 中 target_yaw2real_error 收敛、
   is_fire 随目标进入开火窗口置 1

### 7. 实弹标定

1. 单发点射，记录弹着点
2. 根据弹着点偏差调整参数，重复单发直至命中：

| 弹着点现象 | 调整 |
| --- | --- |
| 沿目标运动方向前后偏 | shoot_speed / shoot_delay 标定 |
| 上下偏 | 标定文件或相机安装角度 |
| 固定方向恒定偏 | buff_radius（量测 R 中心到靶子实际距离） |
| 时准时不准 | 检查黄圈（预测击打点）稳定性，RANSAC 拟合参数 |

3. 弹速标定完成后再连发

## 五、参数表（buff_detector_node）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| model_path | buff.onnx | 模型路径 |
| detector_format | zju | zju=浙大 szu=深大（快一倍） |
| confidence_threshold | 0.5 | 检出率低调低 |
| nms_threshold | 0.4 | NMS |
| buff_mode | 2 | 1=小符 2=大符 |
| buff_color | 1 | 0=蓝 1=红（离线视频是蓝符用 0） |
| buff_radius | 0.7 | R 中心到靶子距离，实车量实际值 |
| shoot_speed | 25.0 | 弹速，算预测击打点用（和 gimbal 一致） |
| shoot_delay | 0.05 | 发射延迟，同上 |
| target_frame | odom | 离线 camera_optical_frame 跳过 tf |
| camera_frame | camera_optical_frame | 相机系 |
| debug_image | false | 发 /buff/debug_image |
| subscribe_compressed | false | 压图订阅 |
| switch_buff_angle | 45.8 | 扇叶 roll 跳变阈值（度） |
| kf_q_pos / kf_q_vel / kf_r_meas | 0.1 / 1.0 / 0.1 | 小符卡尔曼噪声 |
| small/big_max_temp_lost_frames | 20 | 临时丢失帧上限 |
| big_max_converging_frames | 300 | 大符收敛兜底帧 |
| ransac_max_iterations | 200 | RANSAC 迭代 |
| ransac_min/max_omega | 1.884 / 2.0 | 大符角速度范围（rad/s） |
| ransac_min/max_amplitude | 0.78 / 1.045 | 正弦幅值范围 |
| ransac_min_inliers | 100 | 内点数达标进 TRACKING |
| ransac_fit_interval | 5 | 每几帧拟合一次（省 CPU） |

## 六、怎么调车（症状 → 对应参数）

| 症状 | 先调什么 |
| --- | --- |
| 黄圈固定偏一个方向 | buff_radius（量实际半径） |
| 黄圈乱跳 | RANSAC 拟合抖动：看 ransac_max_iterations / min_inliers；跳得厉害先降 ransac_fit_interval 观察 |
| 绿十字偏出靶子 | 检查内参/模型关键点精度；离线先确认假内参是否匹配视频 |
| 十字和黄圈偏离大 | shoot_speed/shoot_delay 与实际不符（正常时偏差≈一个提前量的弧线距离） |
| 进不了 TRACKING | confidence_threshold 调低；视频跑够 20 秒；颜色 buff_color 对不对 |
| 卡（帧率低） | detector_format:=szu（快一倍，需验证）；相机分辨率降一档；GPU 上换 CUDA/TensorRT |
| 打偏（左右） | shoot_speed、shoot_delay 标定 |
| 打偏（上下） | 相机标定/安装角度、弹道参数 |

## 七、代码状态与近期改动（2026-08）

**当前为主**：
- 全链路：检测（zju/szu 可选）→ 筛选 → PnP → tf → 跟踪（小符 KF/大符 RANSAC）→ 预测 → 发布 Target
- 双点：position=目标打击点（gimbal 用），predictive_point=预测击打点（新字段，云台暂未消费）
- 主链路提前量：**保持浙大原版**（buff 发当前点+切向速度，云台弹道解算+直线外推统一算）——不建议改回 buff 侧外推：飞行时间两处不一致会引入误差（已试并回退）

**近期改动**：
- tf 查询用图像时间戳（云台转动时位姿正确）
- 丢失输出清零（armors_num=0）
- RANSAC 降频（每 5 帧拟合，省 CPU）
- 深大模型支持（detector_format 切换）
- PnP 异常位姿过滤（NaN 不污染跟踪器）
- buff_radius / shoot_speed / shoot_delay 参数化
- Target.msg 增加 predictive_point 字段
- 四个 launch 支持 detector_format 切换、实车支持 serial_protocol

**已进行并回退的尝试**：
- R 中心关键点逐帧修正位姿（把黄圈拉飞，已删除）
- 打击点改为 R 中心（R 只是标志，不是靶心，用户确认）
- onnxruntime 多线程/图优化（实测更慢）
- buff 侧正弦外推弹道提前量（飞行时间两处不一致，已回退）

## 八、已知问题

1. **szu 模型离线验证不了**（只在训练过的场地稳，浙大视频上关键点跳动；调曝光实验证明与亮度无关），必须实车
2. CPU 推理是瓶颈（zju 38ms），配置优化实测无效
3. 离线严格对齐验证做不了（假内参、无真实 tf/云台反馈），对齐要到实车做
4. 黄圈是"预测弹着点"的模拟值（fly_time 用距离/弹速估算），和云台精确弹道时间有差异——趋势可参考，精确修正等实车
