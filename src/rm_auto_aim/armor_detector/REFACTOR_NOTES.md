# 2026-09 Latest-frame 低延迟重构说明

## 目标

原结构在 `imageCallback()` 中直接执行传统视觉 / ONNX Runtime / PnP。当算法单帧耗时大于相机周期时，即使 DDS QoS 是 `KEEP_LAST(1)`，executor 仍会被当前 callback 占用，下一张图真正进入 callback 时已经变旧。

本次改为：

```text
ROS image callback
    ↓
latest-frame mailbox (depth = 1, overwrite old)
    ↓
single processing worker
    ↓
Traditional OR Neural
    ↓
PnP
    ↓
publish
```

## 线程模型

- ROS executor：只运行图像接收、参数回调、CameraInfo 等轻量 callback。
- `processing_thread_`：唯一算法线程，串行访问 `Detector`、`NeuralDetector`、`PnPSolver`。
- `frame_mutex_`：只保护 `sensor_msgs::msg::Image::ConstSharedPtr` 与接收时间元数据的交换。
- 不使用 `std::queue`，因此不会在用户态再次建立会积压旧帧的 FIFO。
- worker 忙时，新的相机帧直接覆盖 mailbox 中尚未处理的旧帧。

## 计时定义

- `RX age`：`header.stamp -> imageCallback`。
- `wait`：`imageCallback -> worker start`。
- `work age`：`header.stamp -> worker start`。
- `bridge`：ROS Image -> cv::Mat。
- `detect`：完整检测阶段。
- NN 模式额外拆分 `pre / infer / post / NN total`。
- `PnP`：所有装甲板 solvePnP + 原有 OpenCV projectPoints yaw 重投影优化。
- `pub`：armors + marker 发布。
- `core`：worker start -> 核心结果发布完成，不包含 debug 图绘制。
- `E2E`：`header.stamp -> 核心结果发布完成`。
- `overwritten`：算法忙时被最新帧覆盖的旧帧累计数量。

## 算法保持项

以下算法文件和模型相对上一版没有修改：

- `src/detector.cpp`
- `src/neural_detector.cpp`
- `src/number_classifier.cpp`
- `src/pnp_solver.cpp`
- 对应算法头文件
- `model/shenzhen-0526.onnx`
- `model/mlp.onnx`

因此这次重构主要改变执行调度和性能观测，不改变传统视觉规则、NN 解码逻辑和 PnP 数学流程。

## 性能测试建议

正式测试：

```bash
ros2 run armor_detector armor_detector_node --ros-args \
  -p detector_mode:=neural \
  -p debug:=false \
  -p is_record:=false \
  -p performance_log:=true
```

如果空订阅节点 `RX age` 约 1~2ms，本包重构后日志中的 `RXage` 也应接近这一水平。

当 ONNX CPU 推理约 16~18ms、相机约 100FPS 时，`overwritten` 增长是预期行为：算法无法处理每一帧，但不会排队追旧帧。
