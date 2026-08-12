#include "buff_detector/onnx_buff_detector.hpp"

#include <array>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn.hpp>  // 只复用 blobFromImage，不参与推理
#include <opencv2/imgproc.hpp>

namespace buff_detector {

namespace {

// 推理环境进程内只保留一个，Session 生命周期不能超过它
Ort::Env& get_env() {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "buff_detector");
    return env;
}

}  // namespace

OnnxBuffDetector::OnnxBuffDetector(
    const std::string& model_path,
    const buff_algo::BuffDetectorConfig config
) : config_(config) {
    // 参数校验，和浙大原版保持一致
    if (config_.confidence_threshold < 0.0f || config_.confidence_threshold > 1.0f) {
        throw std::invalid_argument("confidence_threshold must be in [0, 1]");
    }
    if (config_.nms_threshold < 0.0f || config_.nms_threshold > 1.0f) {
        throw std::invalid_argument("nms_threshold must be in [0, 1]");
    }

    try {
        session_ = std::make_unique<Ort::Session>(
            get_env(), model_path.c_str(), Ort::SessionOptions{}
        );
    } catch (const Ort::Exception& error) {
        throw std::runtime_error("failed to load onnx model '" + model_path + "': " + error.what());
    }

    // 查询模型的输入输出名
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
}

OnnxBuffDetector::~OnnxBuffDetector() = default;

std::vector<buff_algo::BuffDetection> OnnxBuffDetector::detect(const cv::Mat& bgr_image) {
    const std::vector<buff_algo::DetectedBuff> detected = detect_with_keypoints(bgr_image);
    std::vector<buff_algo::BuffDetection> detections;
    detections.reserve(detected.size());
    for (const auto& item : detected) {
        detections.push_back(item.detection);
    }
    return detections;
}

std::vector<buff_algo::DetectedBuff> OnnxBuffDetector::detect_with_keypoints(
    const cv::Mat& bgr_image
) {
    if (bgr_image.empty()) {
        throw std::invalid_argument("cannot detect an empty image");
    }
    if (bgr_image.type() != CV_8UC3) {
        throw std::invalid_argument("OnnxBuffDetector expects a CV_8UC3 BGR image");
    }

    // 1. letterbox 等比缩放并填充灰边
    buff_algo::LetterboxInfo info;
    const cv::Mat network_image = buff_algo::letterbox_image(bgr_image, info, input_size_);

    // 2. 归一化到 [0,1] 并转成 RGB，得到网络输入 blob
    cv::Mat blob;
    cv::dnn::blobFromImage(
        network_image,
        blob,
        1.0 / 255.0,
        input_size_,
        cv::Scalar(),
        true,   // swapRB
        false,  // crop
        CV_32F
    );

    // 3. onnxruntime 推理
    const std::array<int64_t, 4> input_shape {
        1, 3, input_size_.height, input_size_.width
    };
    static const Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault
    );
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        reinterpret_cast<float*>(blob.data),
        static_cast<size_t>(blob.total()),
        input_shape.data(),
        input_shape.size()
    );
    const std::array<const char*, 1> input_names {input_name_.c_str()};
    const std::array<const char*, 1> output_names {output_name_.c_str()};
    std::vector<Ort::Value> output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names.data(),
        &input_tensor,
        1,
        output_names.data(),
        1
    );

    // 4. 按浙大格式解码输出 [1, 5040, 26]
    const float* output_data = output_tensors[0].GetTensorData<float>();
    const std::vector<int64_t> output_shape =
        output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    if (output_shape.size() != 3 || output_data == nullptr) {
        throw std::runtime_error("unexpected onnx output shape");
    }
    return buff_algo::decode_detections(
        output_data, output_shape[1], output_shape[2], info, config_
    );
}

}  // namespace buff_detector
