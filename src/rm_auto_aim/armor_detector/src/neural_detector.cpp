// 神经网络装甲板识别实现，见 neural_detector.hpp 的说明。
//
// 推理后端用 onnxruntime（能量机关 buff_detector 里随仓库分发的那份，不额外安装）。
// 没有 onnxruntime 时整个文件退化成空实现，NeuralDetector 构造直接抛异常，
// 传统模式照常编译运行，互不影响。

#include "armor_detector/neural_detector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#ifdef ARMOR_DETECTOR_WITH_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace rm_auto_aim
{
    namespace
    {
#ifdef ARMOR_DETECTOR_WITH_ONNXRUNTIME

        // 模型输入分辨率，深大模型固定 640x640
        constexpr int kDefaultInputW = 640;
        constexpr int kDefaultInputH = 640;
        // 每行输出个数，不是 22 说明模型不匹配，直接报错，避免瞎解码
        constexpr int kOutputCols = 22;
        // 列偏移
        constexpr int kConfCol = 8;
        constexpr int kColorCol = 9; // 9 蓝 10 红 11 灰 12 紫
        constexpr int kGenreCol = 13; // 13 哨兵 14 一号 … 21 基地大

        // 类别 → 本工程使用的装甲板编号 + 大小装甲板
        // 编号沿用 label.txt 里的写法：1~5 / outpost / guard / base，
        // 这样 tracker、串口协议都不用改。
        struct GenreInfo
        {
            const char* number;
            ArmorType type;
        };

        const std::array<GenreInfo, 9> kGenres = {
            {
                {"guard", ArmorType::SMALL}, // 0 哨兵 G
                {"1", ArmorType::LARGE}, // 1 一号（英雄，大装甲板）
                {"2", ArmorType::SMALL}, // 2 二号（工程）
                {"3", ArmorType::SMALL}, // 3 三号
                {"4", ArmorType::SMALL}, // 4 四号
                {"5", ArmorType::SMALL}, // 5 五号
                {"outpost", ArmorType::SMALL}, // 6 前哨站 O
                {"base", ArmorType::SMALL}, // 7 基地小装甲板 Bs
                {"base", ArmorType::LARGE}, // 8 基地大装甲板 Bb
            }
        };

        // 进程内只保留一个 onnxruntime 环境，Session 的生命周期不能超过它
        Ort::Env& getEnv()
        {
            static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "armor_detector");
            return env;
        }

        inline float sigmoid(float x)
        {
            return 1.0F / (1.0F + std::exp(-x));
        }

        // logits 取最大值的下标
        int argmax(const float* data, int len)
        {
            int best = 0;
            for (int i = 1; i < len; ++i)
            {
                if (data[i] > data[best])
                {
                    best = i;
                }
            }
            return best;
        }

        // 用一块自己的内存建输入张量。fp16 / fp32 都走这个口，避免模板特化的坑。
        Ort::Value createTensor(
            const Ort::MemoryInfo& memory_info, void* data, size_t bytes, const std::vector<int64_t>& shape,
            ONNXTensorElementDataType type)
        {
            return Ort::Value::CreateTensor(
                memory_info, data, bytes, shape.data(), shape.size(), type);
        }

        // 用网络角点造一个 Light，保持下游 Armor/PnP 数据结构不变。
        Light makeLight(const cv::Point2f& top, const cv::Point2f& bottom, int color)
        {
            Light light;
            light.top = top;
            light.bottom = bottom;
            light.center = (top + bottom) * 0.5F;
            light.length = cv::norm(top - bottom);
            light.width = 0;
            light.tilt_angle =
                static_cast<float>(std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) / CV_PI * 180.0);
            light.color = color;
            // 兼容 Light 结构；神经网络模式不再做传统角点精修。
            light.pca_top = top;
            light.pca_bottom = bottom;
            return light;
        }

#endif  // ARMOR_DETECTOR_WITH_ONNXRUNTIME
    } // namespace

    struct NeuralDetector::Impl
    {
#ifdef ARMOR_DETECTOR_WITH_ONNXRUNTIME
        std::unique_ptr<Ort::Session> session;
        std::string input_name;
        std::string output_name;
        std::vector<int64_t> input_shape;
        ONNXTensorElementDataType input_type = ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
        int input_w = kDefaultInputW;
        int input_h = kDefaultInputH;
        std::vector<int64_t> tensor_shape;
        cv::Mat resized;
        std::vector<float> buffer_f32;
        std::vector<Ort::Float16_t> buffer_f16;
#else
        int unused = 0;
#endif
    };

    NeuralDetector::NeuralDetector(const std::string& model_path, const NeuralDetectorParams& params)
        : impl_(new Impl), params_(params)
    {
#ifdef ARMOR_DETECTOR_WITH_ONNXRUNTIME
        try
        {
            // 实测默认配置最快，小模型开多线程收益不大
            impl_->session = std::unique_ptr<Ort::Session>(
                new Ort::Session(getEnv(), model_path.c_str(), Ort::SessionOptions{}));
        }
        catch (const Ort::Exception& error)
        {
            throw std::runtime_error("加载神经网络模型失败 '" + model_path + "': " + error.what());
        }

        Ort::AllocatorWithDefaultOptions allocator;
        impl_->input_name = impl_->session->GetInputNameAllocated(0, allocator).get();
        impl_->output_name = impl_->session->GetOutputNameAllocated(0, allocator).get();
        // 注意：TypeInfo 必须先用变量接住，GetTensorTypeAndShapeInfo 返回的是它的借用视图，
        // 写成一句链式调用会读到已经析构的对象
        Ort::TypeInfo type_info = impl_->session->GetInputTypeInfo(0);
        auto input_info = type_info.GetTensorTypeAndShapeInfo();
        impl_->input_type = input_info.GetElementType();
        impl_->input_shape = input_info.GetShape();

        // 输入一般是 [1, 3, H, W]；H/W 从模型里读，避免写死
        if (impl_->input_shape.size() == 4)
        {
            if (impl_->input_shape[2] > 0)
            {
                impl_->input_h = static_cast<int>(impl_->input_shape[2]);
            }
            if (impl_->input_shape[3] > 0)
            {
                impl_->input_w = static_cast<int>(impl_->input_shape[3]);
            }
        }

        impl_->tensor_shape = {1, 3, impl_->input_h, impl_->input_w};
        const size_t input_elements =
            static_cast<size_t>(impl_->input_w) * static_cast<size_t>(impl_->input_h) * 3U;
        if (impl_->input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16)
        {
            impl_->buffer_f16.resize(input_elements);
        }
        else
        {
            impl_->buffer_f32.resize(input_elements);
        }
#else
        (void)model_path;
        throw std::runtime_error("编译时没有找到 onnxruntime，神经网络模式不可用");
#endif
    }

    NeuralDetector::~NeuralDetector() = default;

    bool NeuralDetector::available()
    {
#ifdef ARMOR_DETECTOR_WITH_ONNXRUNTIME
        return true;
#else
        return false;
#endif
    }

    std::vector<Armor> NeuralDetector::detect(const cv::Mat& rgb_img)
    {
        std::vector<Armor> armors;
        last_timing_ = {};
#ifdef ARMOR_DETECTOR_WITH_ONNXRUNTIME
        if (rgb_img.empty() || rgb_img.type() != CV_8UC3)
        {
            return armors;
        }

        const auto total_start = std::chrono::steady_clock::now();
        const int input_w = impl_->input_w;
        const int input_h = impl_->input_h;
        const size_t plane = static_cast<size_t>(input_w) * static_cast<size_t>(input_h);
        const bool fp16 = impl_->input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16;

        // 1) Preprocess: resize + RGB uint8 -> NCHW 0~1. Reuse buffers between frames.
        const auto preprocess_start = std::chrono::steady_clock::now();
        cv::resize(rgb_img, impl_->resized, cv::Size(input_w, input_h), 0, 0, cv::INTER_LINEAR);

        for (int c = 0; c < 3; ++c)
        {
            const size_t channel_offset = static_cast<size_t>(c) * plane;
            for (int y = 0; y < input_h; ++y)
            {
                const uchar* row = impl_->resized.ptr<uchar>(y);
                const size_t row_offset = channel_offset + static_cast<size_t>(y) * input_w;
                for (int x = 0; x < input_w; ++x)
                {
                    const float value = static_cast<float>(row[x * 3 + c]) * (1.0F / 255.0F);
                    const size_t index = row_offset + static_cast<size_t>(x);
                    if (fp16)
                    {
                        impl_->buffer_f16[index] = Ort::Float16_t(value);
                    }
                    else
                    {
                        impl_->buffer_f32[index] = value;
                    }
                }
            }
        }

        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = fp16
                                      ? createTensor(
                                          memory_info, impl_->buffer_f16.data(),
                                          impl_->buffer_f16.size() * sizeof(Ort::Float16_t), impl_->tensor_shape,
                                          ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16)
                                      : createTensor(
                                          memory_info, impl_->buffer_f32.data(),
                                          impl_->buffer_f32.size() * sizeof(float), impl_->tensor_shape,
                                          ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT);
        last_timing_.preprocess_ms = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - preprocess_start).count();

        // 2) ONNX Runtime inference only.
        const auto inference_start = std::chrono::steady_clock::now();
        const char* input_names[] = {impl_->input_name.c_str()};
        const char* output_names[] = {impl_->output_name.c_str()};
        std::vector<Ort::Value> outputs;
        try
        {
            outputs = impl_->session->Run(
                Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
        }
        catch (const Ort::Exception& error)
        {
            throw std::runtime_error(std::string("神经网络推理失败: ") + error.what());
        }
        last_timing_.inference_ms = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - inference_start).count();

        // 3) Decode + threshold + NMS + Armor construction.
        const auto postprocess_start = std::chrono::steady_clock::now();
        const std::vector<int64_t> output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        if (output_shape.size() != 3 || output_shape[2] != kOutputCols)
        {
            throw std::runtime_error("神经网络输出维度不是 [1, N, 22]，模型不匹配");
        }
        const int rows = static_cast<int>(output_shape[1]);
        const float* data = outputs[0].GetTensorData<float>();
        const float scale_x = static_cast<float>(rgb_img.cols) / static_cast<float>(input_w);
        const float scale_y = static_cast<float>(rgb_img.rows) / static_cast<float>(input_h);

        struct Candidate
        {
            cv::Rect box;
            std::array<cv::Point2f, 4> corners; // 左上、左下、右下、右上
            float confidence;
            int color;
            int genre;
        };

        std::vector<Candidate> candidates;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        candidates.reserve(64);
        boxes.reserve(64);
        scores.reserve(64);

        // sigmoid 单调，因此先在 logit 域过滤，可避免对 25200 个候选逐个调用 exp()。
        float logit_threshold = -std::numeric_limits<float>::infinity();
        if (params_.conf_threshold >= 1.0F)
        {
            logit_threshold = std::numeric_limits<float>::infinity();
        }
        else if (params_.conf_threshold > 0.0F)
        {
            logit_threshold = std::log(params_.conf_threshold / (1.0F - params_.conf_threshold));
        }

        for (int row = 0; row < rows; ++row)
        {
            const float* line = data + static_cast<size_t>(row) * kOutputCols;
            if (line[kConfCol] < logit_threshold)
            {
                continue;
            }
            const float confidence = sigmoid(line[kConfCol]);

            // 颜色：0 蓝 1 红 2 灰 3 紫，灰和紫（未激活/混色）直接丢掉。
            const int color_id = argmax(line + kColorCol, 4);
            if (color_id >= 2)
            {
                continue;
            }
            int color = color_id == 0 ? BLUE : RED;
            if (params_.swap_color)
            {
                color = color == BLUE ? RED : BLUE;
            }
            if (color != params_.detect_color)
            {
                continue;
            }

            Candidate candidate;
            candidate.color = color;
            candidate.genre = argmax(line + kGenreCol, 9);
            candidate.confidence = confidence;
            for (int i = 0; i < 4; ++i)
            {
                candidate.corners[i] = cv::Point2f(
                    line[i * 2] * scale_x, line[i * 2 + 1] * scale_y);
            }
            candidate.box = cv::boundingRect(std::vector<cv::Point2f>(
                candidate.corners.begin(), candidate.corners.end()));

            candidates.push_back(candidate);
            boxes.push_back(candidate.box);
            scores.push_back(candidate.confidence);
        }

        if (!candidates.empty())
        {
            std::vector<int> kept;
            cv::dnn::NMSBoxes(boxes, scores, params_.conf_threshold, params_.nms_threshold, kept);
            armors.reserve(kept.size());

            for (int index : kept)
            {
                if (index < 0 || index >= static_cast<int>(candidates.size()))
                {
                    continue;
                }
                const Candidate& candidate = candidates[index];
                const GenreInfo& genre = kGenres[static_cast<size_t>(candidate.genre)];

                if (std::find(
                        params_.ignore_classes.begin(), params_.ignore_classes.end(),
                        std::string(genre.number)) != params_.ignore_classes.end())
                {
                    continue;
                }

                Armor armor(
                    makeLight(candidate.corners[0], candidate.corners[1], candidate.color),
                    makeLight(candidate.corners[3], candidate.corners[2], candidate.color));
                armor.type = genre.type;
                armor.number = genre.number;
                armor.confidence = candidate.confidence;

                std::stringstream result_ss;
                result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
                    << armor.confidence * 100.0F << "%";
                armor.classfication_result = result_ss.str();
                armors.emplace_back(std::move(armor));
            }
        }

        last_timing_.postprocess_ms = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - postprocess_start).count();
        last_timing_.total_ms = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - total_start).count();
#endif  // ARMOR_DETECTOR_WITH_ONNXRUNTIME
        return armors;
    }

    void NeuralDetector::drawResults(cv::Mat& img, const std::vector<Armor>& armors) const
    {
        for (const auto& armor : armors)
        {
            const auto& left = armor.left_light;
            const auto& right = armor.right_light;

            // 网络输出角点即最终交给 PnP 的角点。
            cv::line(img, left.top, left.bottom, cv::Scalar(0, 255, 0), 2);
            cv::line(img, right.top, right.bottom, cv::Scalar(0, 255, 0), 2);
            cv::line(img, left.top, right.top, cv::Scalar(0, 255, 0), 2);
            cv::line(img, right.bottom, left.bottom, cv::Scalar(0, 255, 0), 2);

            cv::putText(
                img, armor.classfication_result, left.top + cv::Point2f(0, -4),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 1);
        }
    }
} // namespace rm_auto_aim
