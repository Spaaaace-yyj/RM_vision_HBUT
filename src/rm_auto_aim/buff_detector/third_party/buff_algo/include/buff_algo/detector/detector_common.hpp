#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include "buff_algo/common/types.hpp"

namespace buff_algo {

struct BuffDetectorConfig {
    float confidence_threshold = 0.5f;
    float nms_threshold = 0.45f;
};

// 解码结果：PnP 四点 + 模型原始的 9 个关键点。
// keypoints[0..7] 是打击板八边形顶点，keypoints[8] 是 R 中心。
struct DetectedBuff {
    BuffDetection detection;
    std::array<Eigen::Vector2f, 9> keypoints{};
};

// 推理预处理（letterbox）后用于坐标还原的信息。
struct LetterboxInfo {
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
    cv::Size original_size;
};

// letterbox 缩放 + 灰边填充，输出尺寸为 input_size 的 BGR 图。
cv::Mat letterbox_image(
    const cv::Mat& image,
    LetterboxInfo& info,
    const cv::Size& input_size
);

// 解码模型输出 [N, 4 + 4 + 18]（固定 5040 行）：
// [cx, cy, width, height, 4 class scores, 9 * (x, y)]。
// 类别顺序：0 = b_inactive, 1 = b_active, 2 = r_inactive, 3 = r_active。
std::vector<DetectedBuff> decode_detections(
    const float* output_data,
    const int64_t num_candidates,
    const int64_t output_columns,
    const LetterboxInfo& info,
    const BuffDetectorConfig& config
);

} // namespace buff_algo
