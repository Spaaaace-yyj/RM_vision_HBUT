#include "buff_algo/detector/detector_common.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace buff_algo {
namespace {

constexpr int kClassCount = 4;
constexpr int kKeypointCount = 9;

float intersection_over_union(const cv::Rect2f& lhs, const cv::Rect2f& rhs) {
    const cv::Rect2f intersection = lhs & rhs;
    const float intersection_area = intersection.area();
    const float union_area = lhs.area() + rhs.area() - intersection_area;
    return union_area > 0.0f ? intersection_area / union_area : 0.0f;
}

Eigen::Vector2f restore_point(
    const float x,
    const float y,
    const float scale,
    const int pad_x,
    const int pad_y,
    const cv::Size& image_size
) {
    const float restored_x = std::clamp(
        (x - static_cast<float>(pad_x)) / scale,
        0.0f,
        static_cast<float>(image_size.width - 1)
    );
    const float restored_y = std::clamp(
        (y - static_cast<float>(pad_y)) / scale,
        0.0f,
        static_cast<float>(image_size.height - 1)
    );
    return {restored_x, restored_y};
}

} // namespace

cv::Mat letterbox_image(
    const cv::Mat& image,
    LetterboxInfo& info,
    const cv::Size& input_size
) {
    info.original_size = image.size();
    info.scale = std::min(
        static_cast<float>(input_size.width) / static_cast<float>(image.cols),
        static_cast<float>(input_size.height) / static_cast<float>(image.rows)
    );

    const int resized_width = static_cast<int>(std::round(image.cols * info.scale));
    const int resized_height = static_cast<int>(std::round(image.rows * info.scale));
    const float half_padding_x = (input_size.width - resized_width) / 2.0f;
    const float half_padding_y = (input_size.height - resized_height) / 2.0f;
    info.pad_x = static_cast<int>(std::round(half_padding_x - 0.1f));
    info.pad_y = static_cast<int>(std::round(half_padding_y - 0.1f));
    const int right = static_cast<int>(std::round(half_padding_x + 0.1f));
    const int bottom = static_cast<int>(std::round(half_padding_y + 0.1f));

    cv::Mat resized;
    cv::resize(image, resized, {resized_width, resized_height}, 0.0, 0.0, cv::INTER_LINEAR);

    cv::Mat padded;
    cv::copyMakeBorder(
        resized,
        padded,
        info.pad_y,
        bottom,
        info.pad_x,
        right,
        cv::BORDER_CONSTANT,
        cv::Scalar(114, 114, 114)
    );
    return padded;
}

std::vector<DetectedBuff> decode_detections(
    const float* output_data,
    const int64_t num_candidates,
    const int64_t output_columns,
    const LetterboxInfo& info,
    const BuffDetectorConfig& config
) {
    if (output_data == nullptr || num_candidates <= 0) {
        return {};
    }
    if (output_columns != 4 + kClassCount + kKeypointCount * 2) {
        throw std::runtime_error(
            "unexpected detector output columns; expected 4 + 4 + 9 * 2"
        );
    }

    struct Candidate {
        DetectedBuff buff;
        cv::Rect2f box;
        int class_id = -1;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(32);

    for (int64_t row_index = 0; row_index < num_candidates; ++row_index) {
        const float* row = output_data + row_index * output_columns;
        const float* class_scores = row + 4;
        const auto best_class = std::max_element(
            class_scores,
            class_scores + kClassCount
        );
        const int class_id = static_cast<int>(best_class - class_scores);
        const float confidence = *best_class;
        if (!std::isfinite(confidence) || confidence < config.confidence_threshold) {
            continue;
        }

        const float center_x = row[0];
        const float center_y = row[1];
        const float width = row[2];
        const float height = row[3];
        if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
            !std::isfinite(width) || !std::isfinite(height) || width <= 0.0f || height <= 0.0f) {
            continue;
        }

        const Eigen::Vector2f top_left = restore_point(
            center_x - width / 2.0f,
            center_y - height / 2.0f,
            info.scale,
            info.pad_x,
            info.pad_y,
            info.original_size
        );
        const Eigen::Vector2f bottom_right = restore_point(
            center_x + width / 2.0f,
            center_y + height / 2.0f,
            info.scale,
            info.pad_x,
            info.pad_y,
            info.original_size
        );
        cv::Rect2f box(
            top_left.x(),
            top_left.y(),
            bottom_right.x() - top_left.x(),
            bottom_right.y() - top_left.y()
        );
        if (box.width <= 0.0f || box.height <= 0.0f) {
            continue;
        }

        std::array<Eigen::Vector2f, kKeypointCount> keypoints;
        bool valid_keypoints = true;
        for (int point_index = 0; point_index < kKeypointCount; ++point_index) {
            const float x = row[8 + point_index * 2];
            const float y = row[8 + point_index * 2 + 1];
            if (!std::isfinite(x) || !std::isfinite(y)) {
                valid_keypoints = false;
                break;
            }
            keypoints[point_index] = restore_point(
                x,
                y,
                info.scale,
                info.pad_x,
                info.pad_y,
                info.original_size
            );
        }
        if (!valid_keypoints) {
            continue;
        }

        DetectedBuff detected_buff;
        detected_buff.keypoints = keypoints;
        BuffDetection& detection = detected_buff.detection;
        detection.color = class_id < 2
            ? static_cast<int>(BuffColor::BLUE)
            : static_cast<int>(BuffColor::RED);
        detection.label = class_id % 2 == 0
            ? static_cast<int>(BuffActivation::INACTIVATE)
            : static_cast<int>(BuffActivation::ACTIVATE);
        detection.confidence = confidence;

        // 0..7 是顺时针排列的八边形顶点。相邻两点的边中点依次为 T/L/B/R。
        detection.t = (keypoints[0] + keypoints[1]) / 2.0f;
        detection.l = (keypoints[2] + keypoints[3]) / 2.0f;
        detection.b = (keypoints[4] + keypoints[5]) / 2.0f;
        detection.r = (keypoints[6] + keypoints[7]) / 2.0f;

        candidates.push_back({detected_buff, box, class_id});
    }

    std::vector<std::size_t> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);
    std::ranges::sort(order, [&](const std::size_t lhs, const std::size_t rhs) {
        return candidates[lhs].buff.detection.confidence >
            candidates[rhs].buff.detection.confidence;
    });

    std::vector<std::size_t> kept;
    for (const std::size_t candidate_index : order) {
        const Candidate& candidate = candidates[candidate_index];
        const bool suppressed = std::ranges::any_of(kept, [&](const std::size_t kept_index) {
            const Candidate& previous = candidates[kept_index];
            return candidate.class_id == previous.class_id &&
                intersection_over_union(candidate.box, previous.box) >= config.nms_threshold;
        });
        if (!suppressed) {
            kept.push_back(candidate_index);
        }
    }

    std::vector<DetectedBuff> detections;
    detections.reserve(kept.size());
    for (const std::size_t index : kept) {
        detections.push_back(candidates[index].buff);
    }
    return detections;
}

} // namespace buff_algo
