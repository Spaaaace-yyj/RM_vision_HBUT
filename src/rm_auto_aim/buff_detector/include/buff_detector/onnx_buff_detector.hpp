#pragma once

// OnnxBuffDetector：能量机关目标检测。
// 接口与浙大 buff_algo::BuffDetector 一致，推理后端换成 onnxruntime，CPU 即可运行。
//
// 支持两种模型格式，detector_format 参数切换，默认 zju 与原来完全一致：
//   zju：浙大 buff.onnx，letterbox 640x384，输出 [1,5040,26]，9 关键点含颜色类别
//   szu：深大 model-0624.onnx，letterbox 640x480，输出 [1,18,6300]，5 关键点
//        （R 字四角 + R 中心），CPU 推理约 19ms，比浙大模型快一倍
//
// 用法：
//   buff_detector::OnnxBuffDetector detector("buff.onnx", {0.5f, 0.4f}, "zju");
//   auto detections = detector.detect_with_keypoints(bgr_image);

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "buff_algo/detector/detector_common.hpp"

namespace Ort {
class Session;
}  // namespace Ort

namespace buff_detector {

// 模型格式：zju=浙大原版，szu=深大 5 点模型
enum class DetectorFormat { ZJU, SZU };

class OnnxBuffDetector {
public:
    // model_path 是模型文件路径，config 是置信度与 NMS 阈值，
    // format 选择模型格式，默认 zju 保持原行为
    OnnxBuffDetector(
        const std::string& model_path,
        buff_algo::BuffDetectorConfig config = {},
        DetectorFormat format = DetectorFormat::ZJU
    );
    // Ort::Session 是不完整类型，析构函数定义在 cpp 里
    ~OnnxBuffDetector();

    // 检测结果，含颜色、激活状态、置信度和 PnP 四点
    std::vector<buff_algo::BuffDetection> detect(const cv::Mat& bgr_image);

    // 检测结果，附带模型输出的关键点，供可视化使用
    std::vector<buff_algo::DetectedBuff> detect_with_keypoints(const cv::Mat& bgr_image);

    cv::Size input_size() const { return input_size_; }
    DetectorFormat format() const { return format_; }

private:
    // 深大模型解码：[1,18,6300] 或 [1,6300,18]，3 类 + 5 关键点(x,y,conf)，
    // 输出转成浙大 BuffDetection（四边中点当方向点）+ 关键点数组
    std::vector<buff_algo::DetectedBuff> decode_szu(
        const float* output_data,
        const std::vector<int64_t>& output_shape,
        const buff_algo::LetterboxInfo& info,
        const cv::Mat& bgr_image);

    buff_algo::BuffDetectorConfig config_;
    DetectorFormat format_ = DetectorFormat::ZJU;
    cv::Size input_size_{640, 384};  // zju 模型输入，宽 640 高 384

    std::unique_ptr<Ort::Session> session_;
    std::string input_name_;
    std::string output_name_;
};

}  // namespace buff_detector
