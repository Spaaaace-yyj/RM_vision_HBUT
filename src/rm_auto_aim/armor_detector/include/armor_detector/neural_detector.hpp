// 神经网络装甲板识别（深大 RobotPilots 开源模型）
//
// 这个文件只为“神经网络模式”服务，不改动原有的传统识别流程：
//   detector_mode = traditional ：走 Detector（传统视觉），本文件不参与
//   detector_mode = neural      ：只走 NeuralDetector（本文件），网络角点直接送入 PnP
//
// 模型：深圳大学 RobotPilots 战队开源的装甲板识别网络（魔改 YOLOv5 + MobileNetV3）
//   仓库 https://github.com/broalantaps/RobotDetectionModel
//   文件 model/shenzhen-0526.onnx（2025-08-09 发布的 0526 版，4.3MB）
//   输入 640x640x3 RGB，0~1，fp16，NCHW；输出 [1, 25200, 22]
//   每行 22 个数：0~7 四角点(左上、左下、右下、右上)，8 置信度(logit)，
//                9~12 颜色(蓝、红、灰、紫)，13~21 类别(哨兵、1~5、前哨站、基地小、基地大)
//   上面这套列定义已用真实图片实测核对（蓝色 3 号装甲板 → 第 9 列最大、类别序号 3），
//   与模型作者的说明文字（红蓝灰紫）不一致，实测为准，详见 README。

#ifndef ARMOR_DETECTOR__NEURAL_DETECTOR_HPP_
#define ARMOR_DETECTOR__NEURAL_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector/armor.hpp"

namespace Ort
{
class Session;
}  // namespace Ort

namespace rm_auto_aim
{

// 神经网络模式的参数，全部由 detector_node 的参数服务器给出
struct NeuralDetectorParams
{
  // 置信度阈值，低于该值的候选直接丢掉（深大原版用 0.65）
  float conf_threshold = 0.65F;
  // NMS 的 IoU 阈值（深大原版用 0.45）
  float nms_threshold = 0.45F;
  // 只保留这个颜色，取值与 armor.hpp 一致：0 蓝 1 红
  int detect_color = BLUE;
  // 红蓝对调。实测 0526 模型第 9 列是蓝、第 10 列是红；
  // 万一实车发现识别出来的红蓝是反的，把这个开关打开即可，不用改代码
  bool swap_color = false;
  // 需要丢弃的类别，沿用传统模式的 ignore_classes 参数（如 base）
  std::vector<std::string> ignore_classes;
};

class NeuralDetector
{
public:
  // 模型加载失败会抛 std::runtime_error，由调用方决定是否退回传统模式
  NeuralDetector(const std::string & model_path, const NeuralDetectorParams & params);
  ~NeuralDetector();

  NeuralDetector(const NeuralDetector &) = delete;
  NeuralDetector & operator=(const NeuralDetector &) = delete;

  // rgb_img 必须是 RGB 三通道（和 detector_node 里 cv_bridge 取出来的格式一致）。
  // 返回的 Armor 已经填好四角点、number、type、confidence、color，
  // 可以直接交给 PnPSolver / 话题发布，字段含义和传统模式完全一样。
  std::vector<Armor> detect(const cv::Mat & rgb_img);

  // 调试可视化：画网络回归的四角点、类别与置信度
  void drawResults(cv::Mat & img, const std::vector<Armor> & armors) const;

  struct Timing
  {
    float preprocess_ms = 0.0F;
    float inference_ms = 0.0F;
    float postprocess_ms = 0.0F;
    float total_ms = 0.0F;
  };

  // 上一帧神经网络各阶段耗时。使用 steady_clock，只统计本进程计算时间。
  const Timing & lastTiming() const { return last_timing_; }

  // 兼容旧调用：这里明确表示纯 Session::Run() 时间，不是整条 NN 链路。
  float lastLatencyMs() const { return last_timing_.inference_ms; }

  // 每帧同步一次可以在线修改的参数（置信度阈值、颜色、忽略类别等）
  void setParams(const NeuralDetectorParams & params) { params_ = params; }

  // 编译时是否带上了 onnxruntime
  static bool available();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  NeuralDetectorParams params_;
  Timing last_timing_;
};


}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NEURAL_DETECTOR_HPP_
