# armor_detector：装甲板识别（传统模式 / 神经网络模式）

本包支持两种识别模式，用一个参数在线切换，互不影响：

| 模式 | 走哪条路 | 说明 |
| --- | --- | --- |
| `traditional`（默认） | `Detector`，二值化 + 找灯条 + PCA + 配对 + MLP 数字分类 | **原有流程一行没改**，行为和以前完全一致 |
| `neural` | `NeuralDetector`，深大 RobotPilots 开源模型 + 传统灯条角点融合 | 新增，神经网络负责“是什么”，传统视觉负责“在哪” |

两种模式最后都输出同一种 `Armor`，所以 PnP、tracker、串口、云台这些下游**都不用改**。

## 1. 怎么切换

### 在线切换（不用重启，推荐调试时用）

```bash
ros2 param set /armor_detector detector_mode neural        # 切到神经网络
ros2 param set /armor_detector detector_mode traditional   # 切回传统识别
```

切换时会打印日志：`检测模式已切换: neural（神经网络）`。
神经网络模型是**第一次用到时才加载**（约 100ms），加载失败会自动退回传统识别并打错误日志，不会把车搞死。

### 启动时就进神经网络模式

```bash
ros2 run armor_detector armor_detector_node --ros-args -p detector_mode:=neural
# launch 里同理：parameters=[detector_params, {'detector_mode': 'neural'}]
```

### 想固化到 bringup/config/params.yaml

在 `/armor_detector: ros__parameters:` 下面加（不加就是默认传统模式）：

```yaml
    detector_mode: traditional      # traditional=传统识别，neural=神经网络
    neural_model_path: ""           # 留空用 share/armor_detector/model/shenzhen-0526.onnx
    neural_conf_threshold: 0.65
    neural_nms_threshold: 0.45
    neural_refine_with_traditional: true
    neural_fallback_traditional: true
    neural_swap_color: false
```

## 2. 参数表

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `detector_mode` | `traditional` | `traditional` 传统识别；`neural` 神经网络 |
| `neural_model_path` | 空 → `model/shenzhen-0526.onnx` | 模型路径，换成 `0708.onnx` 也能跑（输入 dtype 自动识别） |
| `neural_conf_threshold` | 0.65 | 置信度阈值，远距离误识别多就调高，漏检多就调低（0.5~0.6） |
| `neural_nms_threshold` | 0.45 | NMS 的 IoU 阈值，同一个板子出多个框就调小 |
| `neural_refine_with_traditional` | true | 是否用传统灯条 PCA 角点微调 CNN 角点（融合的关键） |
| `neural_fallback_traditional` | true | CNN 这一帧啥也没检出时，是否整帧退回传统流程 |
| `neural_swap_color` | false | 红蓝对调。实测 0526 模型第 9 列是蓝、第 10 列是红；实车若发现颜色反了打开这个开关 |

`detect_color`（0 蓝 1 红）、`ignore_classes`、`debug`、`binary_thres` 这几个老参数在两种模式下都有效。
神经网络模式下 `classifier_threshold` 不再参与判定（数字由网络直接给出），只影响调试信息。

## 3. 模型来源

| 项 | 内容 |
| --- | --- |
| 来源 | 深圳大学 RobotPilots 战队开源：<https://github.com/broalantaps/RobotDetectionModel> |
| 文件 | `Model/0526.onnx`（2025-08-09 发布的 0526 版，4.36MB） |
| 网络 | 魔改 YOLOv5，backbone 用 MobileNetV3，原版用 OpenVINO 在 NUC 上跑 |
| 输入 | `1x3x640x640` **RGB、0~1、fp16** |
| 输出 | `1x25200x22` float32 |
| 训练集 | 约 15K 张 |

输出每行 22 个数的含义（**已用真实图片逐项核对，见第 5 节**）：

```
0~1   左上角点 x,y
2~3   左下角点 x,y
4~5   右下角点 x,y
6~7   右上角点 x,y        （角点顺序：左上起逆时针，640x640 像素坐标）
8     置信度（sigmoid 之后才是概率）
9~12  颜色：蓝、红、灰（未激活）、紫（混色）      ← 只保留与 detect_color 一致的红/蓝，灰紫丢弃
13~21 类别：哨兵G、1号(英雄/大)、2号、3号、4号、5号、前哨站O、基地小、基地大
```

模型类别 → 本工程 `Armor` 的映射（编号沿用 `label.txt`，下游不用改）：

| 模型类别 | number | type |
| --- | --- | --- |
| 哨兵 G | `guard` | small |
| 1 号英雄 | `1` | **large**（大装甲板 0.23m） |
| 2 号工程 / 3、4、5 号步兵 | `2` / `3` / `4` / `5` | small |
| 前哨站 O | `outpost` | small |
| 基地小装甲板 Bs | `base` | small |
| 基地大装甲板 Bb | `base` | **large** |

预处理与深大原版部署保持一致：整幅图**直接拉伸**到 640x640（不做 letterbox，实测两者精度相当，拉伸与原版一致）。

## 4. 神经网络模式做了什么（融合思路）

1. **神经网络**：一帧推理给出装甲板候选 + 四个角点 + 颜色 + 编号，解决“是什么”和传统视觉难处理的误识别（灯条粘连、远处糊成一团）。
2. **传统视觉**：同一帧照常找灯条（二值化 + PCA + 梯度找端点），拿到亚像素级的灯条端点。对每个 CNN 灯条，按中心距离 / 倾角 / 长度比找最近的同一条灯条，用它的端点替换 CNN 回归出来的角点 → PnP 更稳。
   - 匹配阈值、装甲板宽度回退保护见 `RefineParams`，配错灯条会整体回退，不会让位姿飞掉。
   - CNN 原始角点保留在 `light.pca_top / light.pca_bottom`，调试图像里画成**黄色**，融合后的角点画成**绿色**，一眼能看出融合起没起作用。
3. **兜底**：CNN 这一帧一个板子都没检出时，整帧退回传统流程（含 MLP 分类），避免神经网络抖动导致丢帧。
4. 关掉 2、3 就是“纯神经网络模式”：
   ```bash
   ros2 param set /armor_detector neural_refine_with_traditional false
   ros2 param set /armor_detector neural_fallback_traditional false
   ```

调试话题和传统模式共用：`/detector/result_img`、`/detector/number_img`、`/detector/binary_img`（二值图在神经网络模式下只有跑了传统流程的那一帧才发布）、`/detector/debug_lights`、`/detector/debug_armors`。

## 5. 实测记录（2026-09 离线 + 话题联调）

| 项目 | 结果 |
| --- | --- |
| 列语义核对 | 蓝方 3 号装甲板 → 第 9 列（蓝）最大、类别序号 3；`model_infer_example.jpg` 上 4 号板 → 类别序号 4。模型作者文档写的“红蓝灰紫”与实测相反，**以实测为准**（`neural_swap_color` 就是留给这个的保险） |
| 精度 | 深大官方示例图 1440x720：2 个装甲板全部检出，置信度 0.957 / 0.858 |
| 本队远距离图 | `doc/截图 2026-03-10 12-57-13.png`（8mm，4~5m）蓝色 3 号：置信度 0.933 |
| 速度 | 本机 i7-13650HX（20 线程，CPU）：单帧推理 **11.5~13ms**（引用进 640x640），整链路 FPS 还受视频源限制 |
| 话题联调 | `video_pub` 播测试视频 + `armor_detector`：在线切 `neural` 后 `/detector/armors` 正常输出 `number: '3' / type: small` 和 PnP 位姿，切回 `traditional` 正常 |
| 传统模式 | 代码路径未改动，两种模式下 `/detector/*` 调试话题都在 |
| 实车视频（小陀螺） | `blue_rotate_fast.mp4`（1280x1024@60，3813 帧）与 `red_rotate_fast.mp4`（3056 帧）：**每一帧都检出**，蓝 5933 个板 / 红 4758 个板，编号全部正确为 `3`，平均推理 10.9 / 11.3ms |
| 同视频 A/B（蓝方） | 传统模式 100% 帧有检出、1.04 个/帧、处理 34FPS；神经网络模式 100% 帧有检出、**1.55 个/帧**、处理 42FPS —— 快到小陀螺转过侧面那块板时，传统配对规则会漏掉，神经网络补上了 |

## 6. 离线测试程序

不用启动 ROS，直接喂图片或视频（编译产物：`install/armor_detector/lib/armor_detector/test_neural_detector`）：

```bash
source install/setup.bash
cd install/armor_detector/lib/armor_detector

# 图片（默认只认蓝方，--red 认红方）
./test_neural_detector /path/armor.jpg
./test_neural_detector /path/armor.jpg /path/shenzhen-0526.onnx /tmp/out.jpg --red

# 视频：会统计检出帧数、装甲板总数、平均推理耗时
./test_neural_detector /path/armor.avi "" /tmp/out.avi
```

输出里会同时打印 **CNN 角点** 和 **融合角点**，可以直接对比融合前后差多少像素；结果图上黄点=CNN 角点，绿框=最终交给 PnP 的角点。

### 用实车视频看效果（一条命令，自动开看图窗口）

```bash
# 蓝方视频 + 传统模式（默认）：3 秒后自动弹出 rqt_image_view 窗口，画面就是带框的结果图
ros2 launch bringup armor_video_launch.py

# 直接进神经网络模式
ros2 launch bringup armor_video_launch.py detector_mode:=neural

# 红方视频（0=蓝 1=红）
ros2 launch bringup armor_video_launch.py video_path:=red_rotate_fast.mp4 detect_color:=1

# 不想要看图窗口 / 想降帧率
ros2 launch bringup armor_video_launch.py rqt:=false fps:=30
```

窗口里：**绿框**=最终交给 PnP 的角点，**黄点**=CNN 原始角点，左上角文字=编号+置信度。
rqt 顶部的 topic 下拉框还能切到 `/detector/binary_img`（二值图）、`/detector/number_img`（数字小图）看中间结果。
一边跑一边切模式（不用重启）：

```bash
ros2 param set /armor_detector detector_mode neural
ros2 param set /armor_detector detector_mode traditional
```

launch 参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `video_path` | `blue_rotate_fast.mp4` | 视频文件名，必须在 `video_pub/video/` 下（找不到会打印当前可用列表） |
| `detect_color` | 0 | 0 蓝 1 红 |
| `detector_mode` | traditional | 启动时进哪个模式 |
| `fps` | 60 | 回放帧率，检测跟不上就降到 30 |
| `binary_thres` | 140 | 传统二值化阈值 |
| `neural_conf_threshold` | 0.65 | 神经网络置信度阈值 |
| `debug` | true | 发调试图，关了就没有画面可看 |
| `use_tf` | true | 发 `odom→camera_optical_frame` 静态 TF（离线没有云台 TF，不发则 PnP 出不了位姿） |
| `use_tracker` | false | 同时起 `armor_tracker` 看锁定效果 |
| `rqt` | true | 自动开 `rqt_image_view` 看 `/detector/result_img` |

视频要放在 `video_pub` 的 share 目录里（`video_path` 只认该目录下的文件名）：

```bash
cp ~/下载/blue_rotate_fast.mp4 ~/下载/red_rotate_fast.mp4 src/video_pub/video/
colcon build --packages-select video_pub
```

不想用 launch、想手工一条条起，就是下面这几条：

```bash
ros2 run video_pub video_pub_node --ros-args -p video_path:=blue_rotate_fast.mp4 -p fps:=60
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id camera_optical_frame
ros2 run armor_detector armor_detector_node --ros-args \
  -p subscribe_compressed:=false -p detect_color:=0 -p binary_thres:=140 -p debug:=true
ros2 run rqt_image_view rqt_image_view /detector/result_img
ros2 param set /armor_detector detector_mode neural     # 切模式
```

想直接要数字对比（同一段视频各跑 20 秒，自动切模式并统计检出帧/装甲板数/编号）：

```bash
# 先按上面把 video_pub / TF / 检测节点起好，然后
python3 src/rm_auto_aim/armor_detector/tools/ab_compare.py
```

## 7. 编译说明

- 推理后端是 **onnxruntime**，不额外安装：CMake 会先找本包 `third_party/onnxruntime`，找不到就复用 `buff_detector/third_party/onnxruntime`（随仓库分发的那份），也可以 `-DARMOR_ONNXRUNTIME_ROOT=/path/to/onnxruntime` 指定。
- 万一没有 onnxruntime：包照样编译，只是 `detector_mode:=neural` 会报“编译时没有链接 onnxruntime”并退回传统识别。
- 安装时会把 `libonnxruntime.so.1` 装到本包 `lib/` 下（RPATH `$ORIGIN`），和 `buff_detector` 各自一份，互不干扰。

## 8. 实车注意事项 / 排查

- **颜色反了**：`ros2 param set /armor_detector neural_swap_color true`。
- **远距离误识别多**：`neural_conf_threshold` 调到 0.75~0.85（深大自己的建议也是哨兵把阈值调高）；也可以按类别屏蔽，例如 `ignore_classes: ["base"]`。
- **一个板子出好几个框**：`neural_nms_threshold` 从 0.45 调到 0.3。
- **数字偶尔跳**（比如 3/4 之间跳）：tracker 是按 `number` 匹配的，跳变会导致重新锁定；可以先调高置信度阈值，再考虑只用神经网络给颜色、编号仍用 MLP（把 `neural_refine_with_traditional` 保持 true，编号逻辑在 `neural_detector.cpp` 的 `kGenres` 表旁边）。
- **帧率掉**：CPU 推理 12ms 左右，肉眼无感；如果整机吃紧，可以关掉 `debug`，或把 `neural_fallback_traditional` 关掉省一次传统流程。
- **模型只在它训练过的场地最稳**：深大模型和曝光/增益强相关（官方建议低曝光、高增益），实车务必按队里的曝光重新确认阈值。
- **rqt 窗口弹不出来，报 `Could not find Qt binding ... No module named 'PyQt5'`**：`rqt_image_view` 的 shebang 是 `/usr/bin/env python3`，而 PATH 最前面挂的不是系统 python（本机是 `~/.platformio/penv/bin/python3`），那个环境里没有 PyQt5。`armor_video_launch.py` 已经给看图进程单独把 `/usr/bin` 放到 PATH 最前面，直接用 launch 不受影响；手工起的时候写成
  `PATH=/usr/bin:$PATH ros2 run rqt_image_view rqt_image_view /detector/result_img` 即可。
