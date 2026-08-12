#pragma once

// OnnxBuffDetector：能量机关目标检测。
// 接口与浙大 buff_algo::BuffDetector 一致，推理后端换成 onnxruntime，CPU 即可运行。
//
// 用法：
//   buff_detector::OnnxBuffDetector detector("buff.onnx", {0.5f, 0.4f});
//   auto detections = detector.detect_with_keypoints(bgr_image);

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "buff_algo/detector/detector_common.hpp"

namespace Ort {
class Session;
}  // namespace Ort

namespace buff_detector {

class OnnxBuffDetector {
public:
    // model_path 是模型文件路径，config 是置信度与 NMS 阈值
    OnnxBuffDetector(
        const std::string& model_path,
        buff_algo::BuffDetectorConfig config = {}
    );
    // Ort::Session 是不完整类型，析构函数定义在 cpp 里
    ~OnnxBuffDetector();

    // 检测结果，含颜色、激活状态、置信度和 PnP 四点
    std::vector<buff_algo::BuffDetection> detect(const cv::Mat& bgr_image);

    // 检测结果，附带模型输出的 9 个关键点，供可视化使用
    std::vector<buff_algo::DetectedBuff> detect_with_keypoints(const cv::Mat& bgr_image);

    cv::Size input_size() const { return input_size_; }

private:
    buff_algo::BuffDetectorConfig config_;
    cv::Size input_size_{640, 384};  // 与 buff.onnx 输入一致，宽 640 高 384

    std::unique_ptr<Ort::Session> session_;
    std::string input_name_;
    std::string output_name_;
};

}  // namespace buff_detector
