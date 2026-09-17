# image_latency_test

ROS2 C++ 空图像订阅延迟测试节点。

功能：
- 订阅 `/image_raw`
- 使用 sensor-data QoS
- `depth = 1`
- 不做 cv_bridge / OpenCV / 推理
- 统计 `now - msg->header.stamp`
- 每 100 帧打印 current / avg / min / max input age 和接收 FPS

## 编译

把本包放到 ROS2 工作空间的 `src/` 下：

```bash
colcon build \
  --packages-select image_latency_test \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

然后：

```bash
source install/setup.bash
```

## 运行

```bash
ros2 run image_latency_test image_latency_test_node
```

默认订阅：

```text
/image_raw
```

如果你的图像话题名称不同，可以 remap：

```bash
ros2 run image_latency_test image_latency_test_node \
  --ros-args -r /image_raw:=/camera/image_raw
```

输出示例：

```text
Input age: current=2.31 ms | avg=2.48 ms | min=1.72 ms | max=4.83 ms | receive FPS=119.8
```
