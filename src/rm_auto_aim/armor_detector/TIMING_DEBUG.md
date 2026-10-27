# Detector 逐帧耗时与吞吐统计

接口包实际名称是 `auto_aim_interfaces`。新增两个消息，传统识别、ONNX Runtime、
OpenVINO、相机内嵌和 ROS 图像订阅模式都使用同一套统计，不受 `debug` 绘图开关影响。

| 话题 | 消息 | 频率 |
| --- | --- | --- |
| `/detector/debug_timing` | `auto_aim_interfaces/msg/DebugDetectorTiming` | 每个识别周期结束后一次，包含零检出帧和失败周期 |
| `/detector/debug_fps` | `auto_aim_interfaces/msg/DebugDetectorFps` | 1 秒 wall timer 发布一次，空闲时也发布零速率 |

QoS 都是 SensorDataQoS（best effort、volatile、有限历史）。池满拒收的输入没有开始识别周期，
不会制造一条“零耗时结果”；切换模式而失效的异步结果也不发布时间消息，二者在 FPS 话题中分别计数。

## 编译和查看

使用现有 ROS2 Humble 和 OpenVINO C++ SDK 环境，在仓库根目录执行：

```bash
git pull --ff-only
colcon build --symlink-install --packages-up-to armor_detector --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 先启动原来的 detector/launch，再在另一个已 source 的终端查看：
ros2 topic echo /detector/debug_timing --qos-reliability best_effort
ros2 topic echo /detector/debug_fps --qos-reliability best_effort
```

本次修改了消息接口，需要重新编译 `auto_aim_interfaces` 及其使用者。
上面的 `--packages-up-to armor_detector` 会包含接口包；使用整套系统时建议再执行一次原来的全工作区构建。
两个终端都 source 新的 `install/setup.bash`，不要让旧接口定义遮蔽新安装。
构建时确认仍然出现 OpenVINO backend enabled；本修改不改变已有的后端配置。

## 时间基准

`header` 保留原图的时间戳和坐标系；`frame_id` 是 detector 逐输入分配的 uint64 序号，
拒收和丢弃会留下序号空缺。OpenVINO 并行完成后仍使用该帧自己的 header 和计时快照，
不会把下一张图或另一个请求的耗时填进来。

所有 stage 耗时和 FPS 窗口使用 `steady_clock`；原图时间戳、`output_stamp`、
`finished_stamp` 和 `e2e_ms` 使用 ROS clock。仿真需要保证图像与节点使用同一时钟；
ROS 时间跳变/不一致可能使 E2E 出现异常值，而不影响 steady-clock stage 和 FPS 的计时。

| 字段 | 起点 → 终点 |
| --- | --- |
| `e2e_ms` | 原图 `header.stamp` → `/detector/armors` 的 `publish()` 返回 |
| `frame_ready_to_output_ms` | **内嵌模式 RGB 图像转换及 SDK buffer 归还完成** → 结果发布返回 |
| `callback_to_output_ms` | 本帧进入 detector 回调 → 结果发布返回，包含等锁和处理前序结果 |
| `core_ms` | 处理完前序结果后、本帧开始处理 → 结果发布返回；没有输出时截止到周期结束/失败 |
| `cycle_ms` | 回调入口 → marker、调试绘图等完成；不含本条时间消息自身的发布耗时 |
| `result_publish_ms` | 结果 `publish()` 调用本身，便于单独看“处理完成后输出结果”的开销 |

**这里的“发布完成”是本进程 publish() 返回，不是 tracker 收到或处理完成。**
`output_stamp` 对应结果发布后的 ROS 时间，`finished_stamp` 对应整个周期结束时间。
没有有效内参时检测仍可运行，但原有结果发布逻辑不会发布 Armors，
此时 `result_published=false`，所有“到输出”的指标为 NaN，`output_stamp` 为零。
订阅模式的 `frame_ready_to_output_ms` 也为 NaN，因为无法得知上游相机真正完成图像转换的时刻。

内嵌模式沿用现有时间戳语义：在 `CameraGetImageBuffer` 返回原始帧后记录 ROS 时间，
随后进行 ISP/RGB 转换。`e2e_ms` 不包含此前的曝光、传感器读出、传输和可能的 SDK 缓存等待；
`frame_ready_to_output_ms` 进一步排除了本帧的 RGB 转换时间。

## 阶段字段

完整字段和单位也写在 [DebugDetectorTiming.msg](../auto_aim_interfaces/msg/DebugDetectorTiming.msg) 中。

| 字段组 | 含义 |
| --- | --- |
| `camera_wait_ms` / `camera_process_ms` | 内嵌模式取到原始帧前的 SDK 等待、原始帧到 RGB/释放 buffer；订阅模式为 0 |
| `camera_info_publish_ms` | 内嵌模式内参消息发布调用 |
| `input_age_ms` | 原图时间戳到回调入口；内嵌模式包含本帧 RGB 转换及内参发布 |
| `mutex_wait_ms` / `drain_previous_ms` | 等待节点状态锁、提交本帧前消费前序异步结果 |
| `cv_bridge_ms` / `parameter_update_ms` | 图像视图/编码转换、读取运行参数 |
| `model_init_ms` | 后端懒加载/检查，首帧的模型编译时间会单独显示 |
| `record_ms` | 录像转换和写入，未开启则为 0 |
| `traditional_preprocess/lights/match/extract/classify_ms` | 传统灰度二值化、灯条、配对、数字图像提取、数字分类 |
| `traditional_total_ms` | 传统识别完整 detect() 耗时，包含上述子项 |
| `preprocess_ms` / `inference_ms` / `postprocess_ms` | 网络预处理、推理、解码/NMS |
| `neural_total_ms` | 网络 pre + infer + post；不含结果排序/轮询等待 |
| `neural_latency_ms` | 网络 detect() 耗时，或 OpenVINO submit 到按序取出/解码完成 |
| `result_wait_ms` | OpenVINO 完成回调到开始按序解码，包括排序、timer 唤醒和节点忙等待 |
| `pnp_ms` / `pnp_count` | 本帧所有装甲板的 PnP/重投影优化累计耗时和尝试数量 |
| `marker_publish_ms` / `debug_ms` | marker 发布、调试绘图及调试消息发布 |

汇总字段和子阶段存在包含关系，不能把所有字段相加当作 E2E。
传统调试图目前仍在 PnP 前发布，因此开启时其耗时会进入该模式的 E2E；
网络调试图在结果发布后绘制，进入 `cycle_ms`，不进入该帧 E2E。
前一帧的绘图也可能推迟下一帧，由 `mutex_wait_ms`、`drain_previous_ms` 或 `result_wait_ms` 反映。

`completed` 表示周期是否正常走完；`result_published` 表示是否发出了 Armors（空结果也算）。
`traditional_fallback` 表示网络模式退回传统识别，`error` 记录失败原因。
`detected_armors` 是检测出的数量，`published_armors` 是 PnP 成功后发出的数量。

## 每秒吞吐统计

FPS 使用本次 timer 实际经过的 `window_seconds` 计算，调度延迟时不会假定正好 1 秒。
窗口边界可能跨过一帧的提交和完成，因此某一秒 accepted 与 completed 不必完全相等。

| 字段 | 含义 |
| --- | --- |
| `input_fps` | 进入 detector 的图像速率，**不是相机硬件帧率** |
| `accepted_fps` | 真正接收并进入检测/推理的任务速率 |
| `completed_fps` | 正常走完的识别周期，含零检出和成功回退传统；没有内参也可完成识别 |
| `published_fps` | `/detector/armors` 实际发布频率，含空 Armors |
| `rejected_fps` | 请求池满而拒收的速率 |
| `failed_fps` | 有错误的周期；成功回退时会同时计入 completed，不能再用它简单相减 |
| `discarded_fps` | 因模式切换等失效而丢弃的已接收异步结果 |
| `*_total` | 节点启动以来的累计计数，不随窗口清零 |
| `pending` / `capacity` | 当前 OpenVINO 未取回数量与容量，没有初始化该后端时为 0 |

原 `ArmorDetector FPS` 和 2 秒采样的 `NN perf` 日志由上述话题及每秒一次的 `Detector FPS` 日志替代。
需要识别吞吐量时看 `completed_fps`，需要下游实际能收到多少结果时看 `published_fps`。

## 验证

不依赖 ROS 的 FPS 窗口测试：

```bash
cmake -S src/rm_auto_aim/armor_detector/tools/timing -B build/timing_test
cmake --build build/timing_test -j2
ctest --test-dir build/timing_test --output-on-failure
```

编译并 source 新接口后，验证时间戳、64 位帧 ID、NaN 无输出标志及 FPS 序列化：

```bash
python3 src/rm_auto_aim/armor_detector/tools/timing/test_message_roundtrip.py
```

本次已通过 Humble 接口包原生构建（C/C++/Python 类型支持）、上述两类测试，
以及使用真实 Humble、生成消息和 OpenCV 头文件的 detector 编译语法检查。
尚未在相机实机或完整 OpenVINO ROS 运行链路中验证发布频率和额外开销。
