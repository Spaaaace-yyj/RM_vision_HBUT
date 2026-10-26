# OpenVINO CPU 多流异步装甲板识别

保留 `detector_mode: traditional / neural`，在网络模式中新增 `neural_backend: openvino`。
原来的 `onnxruntime` 同步后端仍是默认值。传统 `Detector`、Armor 格式、PnP 和 tracker 算法不变。
本模块专用于仓库的 RobotPilots 角点模型，不是通用 YOLOv8/YOLO11 解码器。

## 1. 版本与安装（Ubuntu 22.04 / NUC12 / C++17）

已验证 **OpenVINO 2024.4.0 和 2025.2.0 Ubuntu22 x86_64 C++ SDK**。
最低版本为 2024.4；已有 2025.2.0 且 C++ ABI=1 的安装可以直接复用，不需要降级。
下面保留 2024.4.0 作为新安装示例；其他更新版本需要重新编译和验证，不保证性能一致。
纯 CPU 不需要 CUDA、TensorRT、Intel GPU 驱动或 NPU。

**不要把 Linux `pip install openvino==2024.4.0` 的 CMake 包直接用于本 C++/ROS 工程。**
该 wheel 使用 `_GLIBCXX_USE_CXX11_ABI=0`，Ubuntu 系统 OpenCV/ROS 使用 ABI=1，
会出现 `cv::putText`、`cv::imread` 等 undefined reference。
Ubuntu C++ SDK 使用 ABI=1；本工程会在 CMake 配置时检查并拒绝 ABI=0。
Python wheel 可单独用于模型转换，不参与 C++ 链接。

在普通终端安装编译依赖：

```bash
sudo apt update
sudo apt install build-essential cmake curl libopencv-dev libeigen3-dev
```

下载官方固定版本，安装到用户目录（不用修改系统 OpenVINO）：

```bash
curl -fL \
  https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.4/linux/l_openvino_toolkit_ubuntu22_2024.4.0.16579.c3152d32c9c_x86_64.tgz \
  -o /tmp/openvino_2024.4.0.tgz
mkdir -p "$HOME/opt/openvino-2024.4.0"
tar -xzf /tmp/openvino_2024.4.0.tgz -C "$HOME/opt/openvino-2024.4.0" --strip-components=1
source "$HOME/opt/openvino-2024.4.0/setupvars.sh"
export OpenVINO_DIR="$HOME/opt/openvino-2024.4.0/runtime/cmake"
```

以后编译和运行均先 `source .../setupvars.sh`。也可使用官方 APT 的 `openvino-2024.4.0` C++ SDK。
切换 SDK 路径后重新配置 CMake，避免沿用之前 wheel 的 `OpenVINO_DIR` 缓存。
如果需要系统运行库，使用 SDK 的 `install_dependencies/install_openvino_dependencies.sh` 安装 CPU 依赖。

### 已安装 2025.2.0 时直接复用

你的 `~/.bashrc` 已经 source SDK 时，新的终端中通常已经有 `INTEL_OPENVINO_DIR` 和 `OpenVINO_DIR`。
确认环境指向同一套 C++ SDK：

```bash
printenv INTEL_OPENVINO_DIR OpenVINO_DIR
export OpenVINO_DIR="$INTEL_OPENVINO_DIR/runtime/cmake"
```

后续编译命令里的 `-DOpenVINO_DIR=...` 改成 `-DOpenVINO_DIR="$OpenVINO_DIR"`，
无需执行 2024.4 安装；后续示例中的 2024.4 setupvars 命令也跳过，保留当前 2025.2 环境。

**曾经编译成功不等于 OpenVINO 后端已经启用。** 旧代码使用
`find_package(OpenVINO 2024.4 QUIET)`，但 OpenVINO 的版本配置要求请求与 SDK 的 major 年份相同，
2025.2 不能满足这个 2024 年份请求，可能静默构建成未启用 OpenVINO 的 stub。
现在改为先发现 SDK，再检查版本是否至少为 2024.4，允许使用已验证的 2025.2。
更新代码后强制重新配置：

```bash
colcon build --symlink-install --packages-up-to armor_detector --cmake-force-configure \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DARMOR_ENABLE_OPENVINO=ON \
  -DOpenVINO_DIR="$OpenVINO_DIR"
```

应看到 `armor_detector: OpenVINO 2025.2.0, CPU async backend enabled`。
可继续检查 `build/armor_detector/CMakeFiles/armor_detector.dir/flags.make` 中存在
`ARMOR_DETECTOR_WITH_OPENVINO`（使用 Makefiles 生成器时），并运行独立测试确认模型实际执行。

## 2. 最快可行性测试：不编译 ROS、不接相机

以下命令均从 **RM_vision_HBUT 仓库根目录** 执行。它只编译网络模块及测试工具，
不要求 ROS、相机 SDK、ONNX Runtime、串口，也不会启动控制链路。

```bash
source "$HOME/opt/openvino-2024.4.0/setupvars.sh"
cmake -S src/rm_auto_aim/armor_detector/tools/openvino \
  -B build/openvino_test -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DOpenVINO_DIR="$HOME/opt/openvino-2024.4.0/runtime/cmake"
cmake --build build/openvino_test -j4
```

先测最大吞吐量，`--fps=0` 表示尽快供给图像，池满时等待并取回结果，不丢输入帧：

```bash
./build/openvino_test/test_openvino_detector \
  --model=src/rm_auto_aim/armor_detector/model/shenzhen-0526.onnx \
  --frames=900 --warmup=30 --fps=0 \
  --threads=10 --streams=0 --requests=0
```

不指定 `--input` 时重复使用 1280×1024 合成 RGB 图像：会真正运行仓库网络、前处理和解码，
适合检查依赖及性能，**不验证目标识别精度**。用真实装甲板图片/视频检查实际场景：

```bash
./build/openvino_test/test_openvino_detector \
  --model=src/rm_auto_aim/armor_detector/model/shenzhen-0526.onnx \
  --input=/absolute/path/armor.jpg \
  --frames=4500 --warmup=30 --fps=150 \
  --threads=10 --streams=0 --requests=0 --csv=/tmp/openvino_150fps.csv
```

图片会重复推理；视频逐帧读取，warmup 会消耗开头帧。`--input=camera:0` 可测试 V4L2 相机，
此时通常用 `--fps=0` 让相机决定供给速度；这不替代迈德威视 SDK/ROS 图像话题测试。
`--color=0/1` 选择蓝/红；`--conf=0.65 --nms=0.45 --swap_color=true` 调整解码。
参数必须写成 `--name=value`。`--help` 显示完整参数。

### 如何判断结果

| 输出 | 含义 |
| --- | --- |
| `threads/streams/requests` | 实际使用的 CPU 线程预算、推理流数和请求池容量 |
| `inputs/accepted/completed` | 实际输入、已接受和已返回的帧数，包含零检出帧 |
| `rejected` | paced 模式中池满而无法接受的帧；会统计，不会偷偷替换老帧 |
| `failed` | 已接受但推理/解码失败的帧 |
| `late_inputs` | 主循环比模拟输入时刻晚超过一个帧周期的次数 |
| `completed FPS` | 完成帧数 / 总墙钟时间，包含最终排空；不是 `1000 / 单帧延迟` |
| `latency p50/p95/p99` | submit 到按序取回结果的延迟，包含前处理、推理、排序等待和解码 |
| `inference p50` | start_async 到完成回调的时间，包含运行时内部等待 |
| `average pre/post` | 前处理和解码的平均耗时 |
| `peak_pending` | 同时未取回的结果/任务数量，最多 requests 个 |

150 FPS 模式按绝对时间模拟每 6.67 ms 一次输入。池满就拒收并计数，不以降低输入速度伪装通过。
`PASS` 只说明本次输入、配置和测试时长内没有拒收、推理错误及明显供给迟到；
**不保证每个输出严格间隔 6.67 ms，也不等于相机 + ROS + PnP 已达到 150 FPS**。
源提前结束而未达到 `--frames` 也不算完整通过。
退出码：0 成功，1 配置/加载/超时错误，2 压测未通过（拒收、推理错误、供给迟到或源提前结束）。
CSV 每帧包含 ID、延迟、前/推理/后处理耗时、检出数及状态。

NUC12 先比较 `--threads=8/10/12`，保留 streams/requests 为 0；
再比较 `--streams=1/2/4` 和适量 requests。请求数不是 CPU 线程数，增大请求池不增加硬件算力，
反而可能增加排队延迟。选择满足输入速度时延迟较低的配置。
纯推理吞吐仅略超 150 FPS 时余量有限；算力不足应再评估 INT8 校准、更小模型或输入尺寸，
不能靠无限排队解决。本模型仍按原始 FP16/FP32 图执行，不会自动做 INT8 量化。

## 3. ROS 集成与模式选择

```bash
source /opt/ros/humble/setup.bash
source "$HOME/opt/openvino-2024.4.0/setupvars.sh"
colcon build --symlink-install --packages-up-to armor_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR="$HOME/opt/openvino-2024.4.0/runtime/cmake"
source install/setup.bash
ros2 run armor_detector armor_detector_node --ros-args \
  -p detector_mode:=neural -p neural_backend:=openvino \
  -p openvino.threads:=10 -p openvino.streams:=0 -p openvino.requests:=0 \
  -p subscribe_compressed:=false -p debug:=false -p is_record:=false
```

相机和 camera_info 仍按原来的 bringup 启动。本包也安装 `test_openvino_detector`，可用
`ros2 run armor_detector test_openvino_detector --model=/absolute/path/model.onnx ...` 运行独立测试。

在 `src/bringup/config/params.yaml` 中选择：

```yaml
/armor_detector:
  ros__parameters:
    detector_mode: neural
    neural_backend: openvino
    openvino:
      threads: 10
      streams: 0
      requests: 0
    subscribe_compressed: false
    debug: false
    is_record: false
```

| 参数 | 默认值 | 是否需要重启 |
| --- | --- | --- |
| `detector_mode` | traditional | 否，可在线 traditional/neural 切换 |
| `neural_backend` | onnxruntime | 是，onnxruntime/openvino |
| `openvino.threads` | 0 | 是，整个编译模型的线程预算；0 自动 |
| `openvino.streams` | 0 | 是，同时执行不同请求的流数；0 自动 |
| `openvino.requests` | 0 | 是，请求和未取回结果的总容量；0 使用运行时建议 |
| 原有置信度/颜色/ignore_classes 参数 | 沿用原值 | 否，每帧提交时保存快照 |

```bash
ros2 param set /armor_detector detector_mode traditional
ros2 param set /armor_detector detector_mode neural
ros2 topic hz /detector/armors
```

图像回调只做前处理与异步提交；2 ms 的 ROS timer 和图像回调都会尝试按序取结果，
然后执行原来的 PnP/发布逻辑。两者在同一默认互斥 callback group，不在 OpenVINO 工作线程中访问 ROS/PnP。
使用完成帧原始 Image/header/timestamp；切换模式后丢弃旧模式尚未返回的结果，避免污染 tracker。
网络正常但零检出时发布空结果；加载失败或某帧推理/解码异常时保留传统识别回退。
池满时不阻塞图像回调、不伪造空观测，明确打印 `OpenVINO full: rejected=...`，说明当前链路跟不上。
DDS/相机本身也可能丢帧（原 image QoS depth=1），池拒收统计不能覆盖这些上游丢帧。
`/detector/armors` 发布还需要收到 camera_info，这是原节点的条件。

没有安装 SDK 时本包仍可编译传统模式和原 ORT 后端。明确禁用可传
`-DARMOR_ENABLE_OPENVINO=OFF`；此时选择 OpenVINO 会记录错误并回退传统模式。

## 4. 模块边界与调用约定

| 文件 | 职责 |
| --- | --- |
| `openvino_detector.hpp/.cpp` | 编译模型、独立请求/输入输出缓冲、异步提交、按序取回、容量限制、关闭等待 |
| `neural_postprocessor.hpp/.cpp` | 两种后端共用的 22 列输出解码、NMS、Armor 构造和绘制 |
| `neural_detector.hpp/.cpp` | 原 ONNX Runtime 同步封装，调用共用解码器 |
| `detector_node.hpp/.cpp` | 模式选择、ROS 图像/时间戳关联、PnP、话题与调试图 |
| `tools/test_openvino_detector.cpp` | 同一模块的离线/模拟固定输入频率测试 |

模型契约：单输入、batch=1、静态 `[1,3,H,W]` FP16/FP32 RGB；单输出 `[1,N,22]`。
仍采用原来的直接 resize、RGB、除以 255，不做 letterbox。
OpenVINO 前处理接口将 host FP32 输入转换到原模型 dtype，并将输出统一到 FP32。
颜色列保持原实测映射：蓝、红、灰、紫；角点保持左上、左下、右下、右上。

API 使用方式：

1. 构造 `OpenVINODetector(model_path, config)`；公开头文件不包含 OpenVINO 或 ROS 头。
2. `submit(rgb, frame_id, params)` 返回 true 才代表被接受；false 代表满，调用方决定等待或丢弃。
3. 用 `tryReceive(result)` 或有超时的 `waitReceive(result, timeout)` 取结果。
4. `result.error` 非空也会推进顺序，不会永远等待这个失败 ID。
5. 接受的 ID 可以不连续；以实际提交顺序组织结果，不依赖 `id+1`。
6. submit 在返回前完成像素复制/前处理；调用方随后可复用相机内存。
7. 输出在取回解码前一直归对应请求所有；完成结果未取回时，该槽不能被下次推理覆盖。
8. 模块支持一个提交线程和一个取结果线程并行运行；析构前调用方先停止并 join 这些线程。
   析构会等待已启动的推理和回调，不允许运行线程访问已经销毁的对象。

`Timing.total_ms` 为 pre + start_async 到完成 + post，不含完成后等待消费者的时间；
`latency_ms` 才包含这段等待。ROS 日志 `core/E2E` 还包括结果排序、PnP 和发布。
按序取回存在慢帧阻塞后续帧的现象，容量有界，但不承诺硬实时截止时间。

## 5. 自动回归测试

测试使用 C++ 生成输入相关的小型模型，不需要 Python、pip 包或额外下载网络：

```bash
cmake -S src/rm_auto_aim/armor_detector/tools/openvino -B build/openvino_test \
  -DBUILD_TESTING=ON \
  -DOpenVINO_DIR="$HOME/opt/openvino-2024.4.0/runtime/cmake"
cmake --build build/openvino_test -j4
ctest --test-dir build/openvino_test --output-on-failure
```

回归检查：角点缩放、颜色/类别、NMS、非有限输出过滤；4 请求/2 流并行时输入覆盖保护、
参数快照、独立提交/消费线程、非连续 ID 按序返回、满池拒收、失败帧推进、错误模型拒绝，以及未取回任务时析构等待。
小型生成模型只测试程序正确性，性能测试必须使用仓库的真实模型。

本次开发环境验证：Ubuntu 24.04、GCC 13、OpenCV 4.6、官方 Ubuntu22 OpenVINO 2024.4.0 / 2025.2.0 C++ SDK；
原 ORT 实现已通过编译和真实模型冒烟运行，未启用 SDK 的 stub 编译也已检查。
3 项 CTest 全部通过；OpenVINO 已用仓库真实模型做 CPU 冒烟运行（2024.4：60 帧；2025.2：30 帧），无推理错误。
当前开发环境没有 ROS2 Humble 和 NUC12 实机，完整 ROS 构建、实机相机输入、识别精度与 150 FPS 结果需在目标机验证。

## 官方参考

- [异步 InferRequest 和完成回调](https://docs.openvino.ai/2024/openvino-workflow/running-inference/integrate-openvino-with-your-application/inference-request.html)
- [CPU 多流与线程调度](https://docs.openvino.ai/2024/openvino-workflow/running-inference/inference-devices-and-modes/cpu-device/performance-hint-and-thread-scheduling.html)
- [吞吐模式与建议请求数](https://docs.openvino.ai/2024/openvino-workflow/running-inference/optimize-inference/high-level-performance-hints.html)
- [Ubuntu C++ SDK 安装](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html)
- [官方 ABI 差异说明](https://openvinotoolkit.github.io/openvino.genai/docs/getting-started/installation/)
