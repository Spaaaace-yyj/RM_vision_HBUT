#include "buff_algo/detector/buff_detector.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <cuda_runtime_api.h>
#include <NvInfer.h>

namespace buff_algo {
namespace {

constexpr int kOutputColumns = 4 + 4 + 9 * 2;
constexpr int kCandidateCount = 5040;

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* message) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::fprintf(stderr, "[TensorRT] %s\n", message);
        }
    }
};

TrtLogger& trt_logger() {
    static TrtLogger logger;
    return logger;
}

std::vector<char> read_file(const std::string& path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream.is_open()) {
        throw std::runtime_error("failed to open engine file: " + path);
    }
    const std::streamsize size = stream.tellg();
    stream.seekg(0, std::ios::beg);
    std::vector<char> data(static_cast<std::size_t>(size));
    if (size > 0 && !stream.read(data.data(), size)) {
        throw std::runtime_error("failed to read engine file: " + path);
    }
    return data;
}

void check_cuda(const cudaError_t error, const char* message) {
    if (error != cudaSuccess) {
        throw std::runtime_error(
            std::string(message) + ": " + cudaGetErrorString(error)
        );
    }
}

} // namespace

BuffDetector::BuffDetector(
    const std::string& engine_path,
    const BuffDetectorConfig config
) : config_(config) {
    if (config_.confidence_threshold < 0.0f || config_.confidence_threshold > 1.0f) {
        throw std::invalid_argument("confidence_threshold must be in [0, 1]");
    }
    if (config_.nms_threshold < 0.0f || config_.nms_threshold > 1.0f) {
        throw std::invalid_argument("nms_threshold must be in [0, 1]");
    }

    runtime_ = nvinfer1::createInferRuntime(trt_logger());
    if (runtime_ == nullptr) {
        throw std::runtime_error("failed to create TensorRT runtime");
    }

    const std::vector<char> engine_data = read_file(engine_path);
    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
    if (engine_ == nullptr) {
        throw std::runtime_error("failed to deserialize engine: " + engine_path);
    }

    context_ = engine_->createExecutionContext();
    if (context_ == nullptr) {
        throw std::runtime_error("failed to create TensorRT execution context");
    }

    input_name_ = engine_->getIOTensorName(0);
    output_name_ = engine_->getIOTensorName(1);

    check_cuda(cudaStreamCreate(reinterpret_cast<cudaStream_t*>(&cuda_stream_)),
               "failed to create CUDA stream");

    allocate_buffers();
}

BuffDetector::~BuffDetector() {
    for (void* buffer : device_buffers_) {
        if (buffer != nullptr) {
            cudaFree(buffer);
        }
    }
    if (cuda_stream_ != nullptr) {
        cudaStreamDestroy(reinterpret_cast<cudaStream_t>(cuda_stream_));
    }
    if (context_ != nullptr) {
        delete context_;
    }
    if (engine_ != nullptr) {
        delete engine_;
    }
    if (runtime_ != nullptr) {
        delete runtime_;
    }
}

void BuffDetector::allocate_buffers() {
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
        const char* name = engine_->getIOTensorName(index);
        const nvinfer1::Dims shape = engine_->getTensorShape(name);
        const nvinfer1::DataType dtype = engine_->getTensorDataType(name);
        if (dtype != nvinfer1::DataType::kFLOAT) {
            throw std::runtime_error("engine tensor must be float32");
        }

        std::size_t elements = 1;
        for (int dimension = 0; dimension < shape.nbDims; ++dimension) {
            elements *= static_cast<std::size_t>(shape.d[dimension]);
        }
        const std::size_t bytes = elements * sizeof(float);

        void* device_buffer = nullptr;
        check_cuda(cudaMalloc(&device_buffer, bytes), "failed to allocate CUDA buffer");
        device_buffers_.push_back(device_buffer);
        buffer_sizes_.push_back(bytes);

        context_->setTensorAddress(name, device_buffer);
    }
}

std::vector<BuffDetection> BuffDetector::detect(const cv::Mat& bgr_image) {
    const std::vector<DetectedBuff> detected = detect_with_keypoints(bgr_image);
    std::vector<BuffDetection> detections;
    detections.reserve(detected.size());
    for (const auto& item : detected) {
        detections.push_back(item.detection);
    }
    return detections;
}

std::vector<DetectedBuff> BuffDetector::detect_with_keypoints(const cv::Mat& bgr_image) {
    if (bgr_image.empty()) {
        throw std::invalid_argument("cannot detect an empty image");
    }
    if (bgr_image.type() != CV_8UC3) {
        throw std::invalid_argument("BuffDetector expects a CV_8UC3 BGR image");
    }

    LetterboxInfo info;
    const cv::Mat network_image = letterbox_image(bgr_image, info, input_size_);

    cv::Mat blob;
    cv::dnn::blobFromImage(
        network_image,
        blob,
        1.0 / 255.0,
        input_size_,
        cv::Scalar(),
        true,
        false,
        CV_32F
    );
    if (blob.total() * sizeof(float) != buffer_sizes_[0]) {
        throw std::runtime_error("input blob size does not match engine input buffer");
    }

    cudaStream_t stream = reinterpret_cast<cudaStream_t>(cuda_stream_);
    check_cuda(
        cudaMemcpyAsync(
            device_buffers_[0],
            blob.data,
            buffer_sizes_[0],
            cudaMemcpyHostToDevice,
            stream
        ),
        "failed to copy input to device"
    );

    if (!context_->enqueueV3(stream)) {
        throw std::runtime_error("TensorRT inference failed");
    }

    std::vector<float> output(kCandidateCount * kOutputColumns);
    check_cuda(
        cudaMemcpyAsync(
            output.data(),
            device_buffers_[1],
            buffer_sizes_[1],
            cudaMemcpyDeviceToHost,
            stream
        ),
        "failed to copy output to host"
    );
    check_cuda(cudaStreamSynchronize(stream), "failed to synchronize CUDA stream");

    return decode_detections(output.data(), kCandidateCount, kOutputColumns, info, config_);
}

} // namespace buff_algo
