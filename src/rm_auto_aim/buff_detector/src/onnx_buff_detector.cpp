#include "buff_detector/onnx_buff_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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

// 深大模型的关键点：0~3 是 R 字四角，4 是 R 字中心。
// 角点顺序模型间可能不同，这里按相对质心的角度分象限，
// 排成 左上、右上、右下、左下，再求四边中点作为上/右/下/左方向点。
std::array<cv::Point2f, 5> sort_szu_keypoints(const std::array<cv::Point2f, 5>& pts) {
    cv::Point2f center(0, 0);
    for (int i = 0; i < 4; ++i) {
        center += pts[i];
    }
    center *= 0.25f;

    // 图像系 y 向下：右上角角度在 (-90°,0]，右下 (0,90°]，左下 (90°,180°]，左上其余
    cv::Point2f tl, tr, br, bl;
    for (int i = 0; i < 4; ++i) {
        const float ang = std::atan2(pts[i].y - center.y, pts[i].x - center.x);
        if (ang > -M_PI_2 && ang <= 0.0f) {
            tr = pts[i];
        } else if (ang > 0.0f && ang <= M_PI_2) {
            br = pts[i];
        } else if (ang > M_PI_2 && ang <= M_PI) {
            bl = pts[i];
        } else {
            tl = pts[i];
        }
    }

    std::array<cv::Point2f, 5> out;
    out[0] = tl;  // 左上
    out[1] = tr;  // 右上
    out[2] = br;  // 右下
    out[3] = bl;  // 左下
    out[4] = pts[4];  // R 中心
    return out;
}

// 深大模型没有颜色类别，在检测框 ROI 里数红蓝亮像素定颜色
int judge_color(const cv::Mat& bgr_image, const std::array<cv::Point2f, 5>& pts) {
    float min_x = 1e9f, min_y = 1e9f, max_x = -1e9f, max_y = -1e9f;
    for (int i = 0; i < 4; ++i) {
        min_x = std::min(min_x, pts[i].x);
        min_y = std::min(min_y, pts[i].y);
        max_x = std::max(max_x, pts[i].x);
        max_y = std::max(max_y, pts[i].y);
    }
    const int x0 = std::max(0, static_cast<int>(min_x));
    const int y0 = std::max(0, static_cast<int>(min_y));
    const int x1 = std::min(bgr_image.cols - 1, static_cast<int>(max_x));
    const int y1 = std::min(bgr_image.rows - 1, static_cast<int>(max_y));
    if (x1 <= x0 || y1 <= y0) {
        return 0;
    }

    long sum_r = 0, sum_b = 0;
    for (int y = y0; y <= y1; y += 2) {
        const auto* row = bgr_image.ptr<cv::Vec3b>(y);
        for (int x = x0; x <= x1; x += 2) {
            sum_r += row[x][2];
            sum_b += row[x][0];
        }
    }
    return sum_r > sum_b ? 1 : 0;  // 1=红 0=蓝
}

}  // namespace

OnnxBuffDetector::OnnxBuffDetector(
    const std::string& model_path,
    const buff_algo::BuffDetectorConfig config,
    const DetectorFormat format
) : config_(config), format_(format) {
    // 参数校验，和浙大原版保持一致
    if (config_.confidence_threshold < 0.0f || config_.confidence_threshold > 1.0f) {
        throw std::invalid_argument("confidence_threshold must be in [0, 1]");
    }
    if (config_.nms_threshold < 0.0f || config_.nms_threshold > 1.0f) {
        throw std::invalid_argument("nms_threshold must be in [0, 1]");
    }

    try {
        // 实测默认配置最快：小模型开多线程/图优化反而更慢
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

    // 输入尺寸按模型格式：浙大 640x384，深大 640x480
    if (format_ == DetectorFormat::SZU) {
        input_size_ = {640, 480};
    }
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

    // 2. 转网络输入。浙大模型：归一化到 [0,1] 并换 RGB；
    //    深大模型：保持 0~255 不归一化，也要换 RGB
    cv::Mat blob;
    if (format_ == DetectorFormat::SZU) {
        cv::Mat rgb;
        cv::cvtColor(network_image, rgb, cv::COLOR_BGR2RGB);
        cv::dnn::blobFromImage(
            rgb, blob, 1.0, cv::Size(), cv::Scalar(), false, false, CV_32F
        );
    } else {
        cv::dnn::blobFromImage(
            network_image, blob, 1.0 / 255.0, input_size_, cv::Scalar(),
            true,   // swapRB
            false,  // crop
            CV_32F
        );
    }

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

    // 4. 解码
    const float* output_data = output_tensors[0].GetTensorData<float>();
    const std::vector<int64_t> output_shape =
        output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    if (output_shape.size() != 3 || output_data == nullptr) {
        throw std::runtime_error("unexpected onnx output shape");
    }

    if (format_ == DetectorFormat::SZU) {
        return decode_szu(output_data, output_shape, info, bgr_image);
    }
    return buff_algo::decode_detections(
        output_data, output_shape[1], output_shape[2], info, config_
    );
}

std::vector<buff_algo::DetectedBuff> OnnxBuffDetector::decode_szu(
    const float* output_data,
    const std::vector<int64_t>& output_shape,
    const buff_algo::LetterboxInfo& info,
    const cv::Mat& bgr_image
) {
    // 深大输出 [1,18,6300] 或 [1,6300,18]：3 类置信度 + 5 关键点 × (x,y,conf)
    const bool nca = output_shape[1] < output_shape[2];
    const long long rows = nca ? output_shape[2] : output_shape[1];
    const long long cols = nca ? output_shape[1] : output_shape[2];
    const auto get = [&](int c, long long a) {
        return nca ? output_data[c * rows + a] : output_data[a * cols + c];
    };

    std::vector<buff_algo::DetectedBuff> results;
    for (long long a = 0; a < rows; ++a) {
        // 3 类：0=未激活 1=小符已激活 2=大符已激活
        int best_cls = -1;
        float max_conf = 0.0f;
        for (int c = 0; c < 3; ++c) {
            const float s = get(c, a);
            if (s > max_conf) {
                max_conf = s;
                best_cls = c;
            }
        }
        if (best_cls < 0 || max_conf < config_.confidence_threshold) {
            continue;
        }

        // 5 个关键点，坐标从网络图还原回原图
        const auto restore = [&](float x, float y) {
            return cv::Point2f(
                std::clamp((x - info.pad_x) / info.scale, 0.0f,
                           static_cast<float>(info.original_size.width - 1)),
                std::clamp((y - info.pad_y) / info.scale, 0.0f,
                           static_cast<float>(info.original_size.height - 1)));
        };
        std::array<cv::Point2f, 5> kpts;
        bool valid = true;
        for (int k = 0; k < 5; ++k) {
            const float x = get(3 + k * 3, a);
            const float y = get(3 + k * 3 + 1, a);
            const float conf = get(3 + k * 3 + 2, a);
            if (x < 0.0f || y < 0.0f || conf < 0.1f) {
                valid = false;
                break;
            }
            kpts[k] = restore(x, y);
        }
        if (!valid) {
            continue;
        }

        // 四角按角度排成 左上 右上 右下 左下，取四边中点当方向点
        const std::array<cv::Point2f, 5> sorted = sort_szu_keypoints(kpts);
        const cv::Point2f top = (sorted[0] + sorted[1]) * 0.5f;     // 上
        const cv::Point2f right = (sorted[1] + sorted[2]) * 0.5f;   // 右
        const cv::Point2f bottom = (sorted[2] + sorted[3]) * 0.5f;  // 下
        const cv::Point2f left = (sorted[3] + sorted[0]) * 0.5f;    // 左

        buff_algo::DetectedBuff result;
        result.detection.t = Eigen::Vector2f(top.x, top.y);
        result.detection.l = Eigen::Vector2f(left.x, left.y);
        result.detection.b = Eigen::Vector2f(bottom.x, bottom.y);
        result.detection.r = Eigen::Vector2f(right.x, right.y);
        // 深大类别：0=未激活（要打的），1/2=已激活
        result.detection.label = best_cls == 0
            ? static_cast<int>(buff_algo::BuffActivation::INACTIVATE)
            : static_cast<int>(buff_algo::BuffActivation::ACTIVATE);
        result.detection.confidence = max_conf;
        result.detection.color = judge_color(bgr_image, sorted);
        // keypoints 前 5 个放深大的 5 点，画调试图用
        for (int k = 0; k < 5; ++k) {
            result.keypoints[k] = Eigen::Vector2f(kpts[k].x, kpts[k].y);
        }
        results.push_back(result);
    }
    return results;
}

}  // namespace buff_detector
