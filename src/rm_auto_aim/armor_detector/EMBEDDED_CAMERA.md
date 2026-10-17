# 内嵌相机模式（`use_embedded_camera`）

> 提交：`1dc9247`　分支：`dev_cameraSDK_move`
> 目的：验证"把相机 SDK 直接塞进检测节点"能不能降低延迟。

---

## 1. 背景

实车观察到的现象：

- 检视图左上角 `Latency` 约 **20 ms**
- 检测帧率约 **100 FPS**（≈ 10 ms/帧）
- 两个数对不上：如果每帧只花 10 ms，延迟不该是 20 ms

当时的猜测是"相机 SDK → ROS 话题"这一跳太慢，于是有了这次改动：
**让检测节点自己调相机 SDK，不经过 `/image_raw` 话题。**

---

## 2. 结论先说

这个模式**已经实现了**，但它的收益是 **约 0.6 ms**，不是那 10 ms。

原因（读代码 + 实测得出）：

| 事实 | 证据 |
| --- | --- |
| 相机节点和检测节点**本来就在同一个进程** | `armor_launch.py` 用同一个 `ComposableNodeContainer` |
| 传图**本来就是零拷贝** | 两个节点都开了 `use_intra_process_comms`；检测端 `cv_bridge::toCvShare` 不复制 |
| 实测传图耗时 | 组 Image 消息 0.56 ms + publish 0.003 ms + 回调交付 0.024 ms ≈ **0.6 ms** |

所以"塞进同一个进程"这件事架构上已经做了一大半。这个改动能省的，只是 publish/订阅分发那 0.6 ms。

**那 20 ms 里更多的部分，大概率在队列排队和检测本身，不在传图。**
详见文末"下一步"。

---

## 3. 架构对比

```
【改前】两个节点 + ROS 话题
mv_camera_node（独立节点）
  CameraGetImageBuffer → CameraImageProcess → image_msg_.data
  → publish(/image_raw) ──[ROS 话题]──► armor_detector.imageCallback
                                        → cv_bridge → detectArmors → PnP → publish

【改后】use_embedded_camera:=true
armor_detector_node（自己拿相机）
  capture_thread_():
    CameraGetImageBuffer → CameraImageProcess → 拼 sensor_msgs::Image
    → 直接调用 imageCallback()      ← 少的就是 publish/订阅这一跳
    → cv_bridge → detectArmors → PnP → publish   ← 完全复用
```

关键设计：**识别逻辑一行没改**。
`captureLoop()` 只是拼一个 `sensor_msgs::msg::Image`，然后调用原来的 `imageCallback`，
所以 `detectArmors`、PnP、marker、发布全部走原路径，行为一致。

---

## 4. 改动清单

3 个文件，+179 行：

| 文件 | 改动 |
| --- | --- |
| `include/armor_detector/detector_node.hpp` | `CameraApi.h` include（条件编译）、析构声明、`initEmbeddedCamera()` / `captureLoop()`、相机成员变量 |
| `src/detector_node.cpp` | 构造函数加 `use_embedded_camera` 开关、析构（停线程 + `CameraUnInit`）、两个新函数 |
| `CMakeLists.txt` | 按 amd64/arm64 自动链接 `mvsdk`，找不到只告警 |

### 新增的参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `use_embedded_camera` | `false` | `true` = 检测节点自己调 SDK；`false` = 订阅 `/image_raw`（原行为） |
| `exposure_time` | 5000 | 曝光（微秒），和 `mv_camera` 同名同义 |
| `analog_gain` | 相机当前值 | 模拟增益 |
| `rgb_gain.r/g/b` | 相机当前值 | RGB 增益 |
| `flip_image` | `false` | 图像翻转 |

这些参数名和 `mindvision_camera` 包保持一致，可以沿用同一份 `params.yaml`。

### CMake 部分

```cmake
set(MVSDK_ROOT ".../src/driver/mindvision_camera/mvsdk")
# aarch64/arm64 → lib/arm64，其余 → lib/amd64
target_compile_definitions(${PROJECT_NAME} PUBLIC ARMOR_DETECTOR_WITH_CAMERA)
target_link_libraries(${PROJECT_NAME} "${MVSDK_LIBDIR}/libMVSDK.so")
```

如果 SDK 不存在：只打 `WARNING`，`use_embedded_camera` 无效，其余功能照常编译运行。

---

## 5. 怎么用

### 编译

```bash
cd /home/twy/RM_vision_HBUT
source /opt/ros/humble/setup.bash
colcon build --packages-select armor_detector
source install/setup.bash
```

编完看日志里有没有这一行：

```
-- armor_detector: 内嵌相机模式可用（.../mvsdk/lib/amd64）
```

### 运行（内嵌模式）

**不要启动 `mv_camera` 节点**，两个进程抢同一台相机会初始化失败。

```bash
ros2 run armor_detector armor_detector_node --ros-args \
  -p use_embedded_camera:=true \
  -p debug:=true
```

### 运行（原来的订阅模式，行为不变）

```bash
ros2 run mindvision_camera mindvision_camera_node
ros2 run armor_detector armor_detector_node --ros-args -p debug:=true
```

### A/B 对比测延迟

两种方式各跑 30 秒，看 `rqt_image_view /detector/result_img` 左上角的
`Latency: xx.xx ms` 和日志里的 `ArmorDetector FPS: nn`。

```bash
# A：原方式
ros2 run mindvision_camera mindvision_camera_node &
ros2 run armor_detector armor_detector_node --ros-args -p debug:=true

# B：内嵌方式
ros2 run armor_detector armor_detector_node --ros-args \
  -p use_embedded_camera:=true -p debug:=true
```

---

## 6. 验证记录（无相机的本机测试）

| 用例 | 结果 |
| --- | --- |
| 编译（`-Wall -Werror`） | ✅ 通过，`libMVSDK.so` 链接成功 |
| 默认模式（`video_pub` 回放 + 订阅） | ✅ 无 ERROR，FPS 30~44（分进程回放），功能正常 |
| `use_embedded_camera:=true` 但没接相机 | ✅ 打 `ERROR: use_embedded_camera:=true 但没找到相机`，**节点不崩溃、继续存活** |

⚠️ 真机延迟对比**还没做**——需要接上相机才能测。

---

## 7. 能省多少（微基准实测）

1280×1024 RGB8（3.75 MB/帧），300 帧平均：

| 阶段 | 耗时 |
| --- | --- |
| 相机侧 `CameraImageProcess`（Bayer→RGB8） | 0.54 ms ← 两种方案都躲不掉 |
| 组 `Image` 消息（含 `data` 拷贝 3.75 MB） | 0.56 ms |
| `publish()` 调用本身 | 0.003 ms |
| publish → 回调开始（跨线程交付） | 0.024 ms |
| 回调内 `toCvShare` 取图 | 0.002 ms |
| **合计（现状 intra-process 传图）** | **≈ 0.6 ms** |
| **合计（内嵌模式）** | **≈ 0 ms**（只剩 ISP 那 0.54 ms） |

对照：**跨进程**时才有的序列化开销 —— 0.23 + 0.89 = **1.12 ms**。

所以：

- 同进程 intra-process：内嵌最多省 **0.6 ms**
- 如果原来是分进程：内嵌最多省 **0.6 + 1.12 ≈ 1.8 ms**

参考值：传统识别 `detect()` 约 **3 ms**；60 FPS 一帧的预算是 **16.7 ms**。

---

## 8. 注意事项 / 坑

1. **`-Wall -Werror`**：只取不用的变量会直接编译失败。代码里已经用 `(void)exposure_line_time` 处理。
2. **`CameraReleaseImageBuffer` 必须在 `imageCallback` 之前调**。放在检测之后的话，检测那几毫秒内核缓冲一直被占，相机侧会阻塞，帧率反而掉。
3. **别同时启动 `mv_camera` 节点**，会抢同一台相机。
4. **`img_msg->data` 要按 `w*h*3` resize**，`CameraImageProcess` 会按这个尺寸往外写，buffer 小了会内存越界。
5. **曝光/增益参数要和原来一致**，否则画面亮度变了，会误判成"集成导致识别变差"。
6. **`install/lib/libMVSDK.so` 可能是符号链接**（指向源码目录）。车上重新 `colcon build` 没问题；如果要把 `install/` 整个拷走，记得把真实 `.so` 一起带上。

---

## 9. 下一步（这才是重点）

内嵌相机做完之后，**20 ms 的构成仍然要测**，建议顺序：

1. **先量清楚那 20 ms**：在检测节点里加插桩，分别记录
   - `header.stamp` → 回调开始（传图 + 排队）
   - 回调开始 → 检测结束（检测本身）
2. **查 SDK 缓冲积压**：`tSdkFrameHead` 里有 `UINT uiTimeStamp`（本帧采集时间，单位 0.1 ms）。
   现在的代码打时间戳用的是 `this->now()`（取图时刻），
   两者相减就是**这张图在 SDK 内核缓冲里排了多久**：

   ```cpp
   RCLCPP_INFO(logger, "SDK 缓冲积压 %.1f ms",
       this->now().seconds() * 1000.0 - frame_info_.uiTimeStamp * 0.1);
   ```

   如果这个值几十毫秒，说明真凶是**取图策略**（应改成只取最新帧、丢旧帧），
   而不是 ROS 传图——改架构解决不了。
3. **看 Latency 曲线的形状**：
   - 稳定在 20 ms → 稳定积压 1 帧
   - 一直往上爬 → 消费跟不上产出（相机帧率设太高）
   - 10~25 乱跳 → 调度抖动

---

## 10. 移植到 `dev_EKF`（神经网络版）

这份改动基于 `dev_cameraSDK_move`（上游 main），**没有神经网络**。
如果实车跑的是 `dev_EKF`，需要移植：

```bash
git switch dev_EKF
git cherry-pick 1dc9247
```

`detector_node.cpp` 在 `dev_EKF` 上多了神经网络相关代码，可能出现冲突。
冲突时保留两边的改动即可：
- 保留 `dev_EKF` 的 `initNeuralParams()` / `detectArmorsByNeural()`
- 加入内嵌相机的 `initEmbeddedCamera()` / `captureLoop()`
- 构造函数里两者共存：先 `use_embedded_camera` 判断图像来源，神经网络模式照常由 `detector_mode` 控制
