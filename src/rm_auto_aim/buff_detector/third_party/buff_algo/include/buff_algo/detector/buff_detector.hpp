#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "buff_algo/detector/detector_common.hpp"

namespace nvinfer1 {
class IRuntime;
class ICudaEngine;
class IExecutionContext;
} // namespace nvinfer1

namespace buff_algo {

// buff.engine 的 TensorRT 推理与解码器。
// 要求 engine 输入 [1, 3, 384, 640]、输出 [1, 5040, 26]。
class BuffDetector {
public:
    explicit BuffDetector(
        const std::string& engine_path,
        BuffDetectorConfig config = {}
    );

    ~BuffDetector();

    std::vector<BuffDetection> detect(const cv::Mat& bgr_image);

    // 额外返回模型原始关键点，供可视化使用。
    std::vector<DetectedBuff> detect_with_keypoints(const cv::Mat& bgr_image);

    cv::Size input_size() const { return input_size_; }

private:
    void allocate_buffers();

    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    void* cuda_stream_ = nullptr;

    std::string input_name_;
    std::string output_name_;
    std::vector<void*> device_buffers_;
    std::vector<std::size_t> buffer_sizes_;

    BuffDetectorConfig config_;
    cv::Size input_size_{640, 384};
};

} // namespace buff_algo
