# detector 内嵌 MindVision SDK 取流

将 `main`（`d331e1a9`）的 `use_embedded_camera` 接入方式同步到 `dev_EKF`，
同时兼容传统识别、ONNX Runtime 和 OpenVINO CPU 多流异步推理。

`use_embedded_camera: false`（默认）仍订阅 `/image_raw` 与 `/camera_info`，
原实车 launch、视频回放入口无需改动。`true` 时 detector 自己打开 SDK，
采集线程直接调用检测入口，不发布或订阅 `/image_raw`，不启动独立相机节点。
内参用 `camera_info_manager` 加载，并按每帧时间戳发布 `/camera_info`。
调试图仍通过 `/detector/result_img` 查看，需要 `debug: true`。

## 编译

以下命令从仓库根目录执行。沿用已有 OpenVINO 2025.2.0 C++ SDK 环境即可；
OpenVINO 安装、版本和 ABI 说明见 [OPENVINO.md](OPENVINO.md)。

```bash
sudo apt install ros-humble-camera-info-manager
source /opt/ros/humble/setup.bash
# 如果终端尚未初始化 OpenVINO，先 source 你实际安装位置的 setupvars.sh。
colcon build --symlink-install --packages-up-to armor_detector bringup \
  --cmake-force-configure --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DARMOR_ENABLE_CAMERA=ON -DARMOR_ENABLE_OPENVINO=ON \
  -DOpenVINO_DIR="$INTEL_OPENVINO_DIR/runtime/cmake"
source install/setup.bash
```

CMake 应同时显示：

```text
armor_detector: embedded MindVision camera enabled (.../mvsdk/lib/amd64)
armor_detector: OpenVINO 2025.2.0, CPU async backend enabled
```

SDK 默认使用 `src/driver/mindvision_camera/mvsdk` 中随仓库分发的头文件和库，
x86_64 使用 `lib/amd64`，aarch64 使用 `lib/arm64`。自定义位置可传 `-DMVSDK_ROOT=/path/to/mvsdk`。
安装时会携带 `libMVSDK.so`，不会覆盖已有 ONNX Runtime 的 RPATH。
没有 SDK 或 `-DARMOR_ENABLE_CAMERA=OFF` 时仍可编译订阅模式；
此时显式启用内嵌相机会报错退出，避免启动后一直没有图像。

## 只测 detector

```bash
ros2 launch bringup armor_embedded_launch.py \
  detector_only:=True detector_mode:=neural neural_backend:=openvino
```

这个模式只启动 detector，不启动 tracker、串口、TF 发布、控制器或 overlay。
启动日志确认 `Embedded MindVision camera: SDK -> detector` 及实际 OpenVINO threads/streams/requests。
相机参数从 `bringup/config/params.yaml` 的 `/mv_camera` 段读取，
检测参数从 `/armor_detector` 段读取；没有把曝光/增益改为另一套默认值。

```bash
ros2 topic hz /detector/armors
ros2 param set /armor_detector detector_mode traditional
ros2 param set /armor_detector detector_mode neural
```

`/detector/armors` 包括零检出帧，可以观察完成频率；无 TF 时不能据此验证目标位姿正确性。
现在用 `/detector/debug_fps` 每秒分别发布输入、接收、完成、发布和拒收频率，
用 `/detector/debug_timing` 逐帧发布各阶段耗时。二者都不受 debug 绘图开关影响。
具体计时边界见 [TIMING_DEBUG.md](TIMING_DEBUG.md)。原图时间戳从 SDK 取到原始帧时记录，
包含随后 ISP 转换，不是传感器硬件曝光时间戳。
性能对比时保持曝光、模型、CPU 线程/流/请求数一致，并关闭 debug 和录像。
嵌入 SDK 不增加 CPU 推理算力，是否达到相机帧率仍需目标机实测。

## 实车整链路

```bash
ros2 launch bringup armor_embedded_launch.py detector_mode:=neural neural_backend:=openvino
```

沿用 `main` 的实车入口，同时启动现有 tracker、串口、云台控制与 TF 等节点。
其他参数包括 `use_serial`、`use_sim_time`、`tracker_overlay_debug`；
省略 detector_mode/neural_backend 时读取 YAML 中的值。
不要同时运行原相机 launch 或另一个内嵌节点，它们会占用同一台相机。

也可直接启动节点，但此方式不会读取 `/mv_camera` 配置，须自行传入相机参数：

```bash
ros2 run armor_detector armor_detector_node --ros-args \
  -p use_embedded_camera:=true -p detector_mode:=neural -p neural_backend:=openvino \
  -p camera_info_url:=package://bringup/config/camera_info.yaml \
  -p exposure_time:=4000 -p debug:=false -p is_record:=false
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_embedded_camera` | `false` | 输入源选择，重启生效 |
| `camera_info_url` | `package://bringup/config/camera_info.yaml` | 内参文件；加载失败或焦距无效时退出 |
| `camera_name` | `mv_camera` | camera_info_manager 名称 |
| `frame_id` | `camera_optical_frame` | 图像和内参使用同一坐标系 |
| `exposure_time` | `5000` | 微秒；launch 以 `/mv_camera` 配置覆盖 |
| `analog_gain` | `-1` | 保留 SDK 当前值，或显式设置模拟增益 |
| `rgb_gain.r/g/b` | 各为 `-1` | 未指定的通道保留 SDK 当前值 |
| `gamma` / `saturation` | 各为 `-1` | 保留 SDK 当前值，或显式设置 |
| `flip_image` | `false` | 与原驱动一致，旋转 180° |

相机参数仅启动时应用，需要重启；`detector_mode`、debug 和已有网络阈值仍可在线调整。
`subscribe_compressed` 仅在订阅模式生效。`openvino.*` 和 `neural_backend` 仍需重启。

## 与 main 同步时对异步链路的适配

- `EmbeddedCamera` 单独管理 SDK 句柄、RGB 转换、翻转及原始 buffer 生命周期，公开头文件不暴露 SDK 类型。
- 不能像同步版本一直覆写同一个 Image。OpenVINO 在途帧保留自己的像素和 header，
  只有没有其他持有者时才复用当前图像；请求池仍有容量上限，池满计数拒收，不无限排队。
- SDK buffer 在进入检测前归还，转换、尺寸检查或时间戳回调抛异常时也归还。
- 采集入口、结果定时器、内参回调及 debug/模式事件回调共用互斥锁，保护 pending 表、PnP 与发布状态；
  运行时内部多个 InferRequest 仍可并行，不在图像回调中等待推理完成。
- 采集线程最后启动，确保节点构造完成；退出先停止并 join 采集线程，再关闭相机和推理请求。

## 无相机回归测试

```bash
cmake -S src/rm_auto_aim/armor_detector/tools/camera -B build/camera_test
cmake --build build/camera_test -j2
ctest --test-dir build/camera_test --output-on-failure
```

测试使用仓库真实 `CameraApi.h` 声明和模拟 SDK 函数，覆盖相机参数、RGB 输出独立性、
时间戳顺序、成功/超时/错误路径、异常时释放 buffer、初始化失败释放句柄，以及禁用 SDK 时明确报错。
无需相机、ROS、OpenCV 或 OpenVINO。

本次已编译相机封装并通过以上两项测试及 launch Python 语法检查；
用 Humble 的真实 launch API 验证了 YAML 合并、后端覆盖、detector_only 条件和延迟加载 TF 描述。
另已链接仓库实际 `libMVSDK.so` 验证无设备时初始化明确报错。
当前验证环境没有完整 ROS2 Humble、OpenVINO SDK 和相机硬件，
尚未验证整包 colcon 构建、实机取流、ROS 整链路或 NUC 性能。
