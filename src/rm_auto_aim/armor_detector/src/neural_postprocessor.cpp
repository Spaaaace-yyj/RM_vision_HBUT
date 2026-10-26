#include "armor_detector/neural_postprocessor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
namespace rm_auto_aim {
namespace {
// 每行输出个数，不是 22 说明模型不匹配，直接报错，避免瞎解码
constexpr int kOutputCols = 22;
// 列偏移
constexpr int kConfCol = 8;
constexpr int kColorCol = 9;   // 9 蓝 10 红 11 灰 12 紫
constexpr int kGenreCol = 13;  // 13 哨兵 14 一号 … 21 基地大

// 类别 → 本工程使用的装甲板编号 + 大小装甲板
// 编号沿用 label.txt 里的写法：1~5 / outpost / guard / base，
// 这样 tracker、串口协议都不用改。
struct GenreInfo {
  const char* number;
  ArmorType type;
};

const std::array<GenreInfo, 9> kGenres = {{
    {"guard", ArmorType::SMALL},    // 0 哨兵 G
    {"1", ArmorType::LARGE},        // 1 一号（英雄，大装甲板）
    {"2", ArmorType::SMALL},        // 2 二号（工程）
    {"3", ArmorType::SMALL},        // 3 三号
    {"4", ArmorType::SMALL},        // 4 四号
    {"5", ArmorType::SMALL},        // 5 五号
    {"outpost", ArmorType::SMALL},  // 6 前哨站 O
    {"base", ArmorType::SMALL},     // 7 基地小装甲板 Bs
    {"base", ArmorType::LARGE},     // 8 基地大装甲板 Bb
}};

inline float sigmoid(float x) { return 1.0F / (1.0F + std::exp(-x)); }

// logits 取最大值的下标
int argmax(const float* data, int len) {
  int best = 0;
  for (int i = 1; i < len; ++i) {
    if (data[i] > data[best]) {
      best = i;
    }
  }
  return best;
}

// 用网络角点造一个 Light，保持下游 Armor/PnP 数据结构不变。
Light makeLight(const cv::Point2f& top, const cv::Point2f& bottom, int color) {
  Light light;
  light.top = top;
  light.bottom = bottom;
  light.center = (top + bottom) * 0.5F;
  light.length = cv::norm(top - bottom);
  light.width = 0;
  light.tilt_angle = static_cast<float>(
      std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) /
      CV_PI * 180.0);
  light.color = color;
  // 兼容 Light 结构；神经网络模式不再做传统角点精修。
  light.pca_top = top;
  light.pca_bottom = bottom;
  return light;
}

}  // namespace
std::vector<Armor> decodeNeuralOutput(const float* data, std::size_t rows,
                                      cv::Size input_size, cv::Size image_size,
                                      const NeuralDetectorParams& params) {
  if (!data || input_size.width <= 0 || input_size.height <= 0) {
    throw std::invalid_argument("Invalid neural output or input size");
  }
  if (!std::isfinite(params.conf_threshold) || params.conf_threshold < 0 ||
      params.conf_threshold > 1 || !std::isfinite(params.nms_threshold) ||
      params.nms_threshold < 0 || params.nms_threshold > 1) {
    throw std::invalid_argument(
        "Neural thresholds must be finite and in [0,1]");
  }
  const float scale_x = static_cast<float>(image_size.width) / input_size.width;
  const float scale_y =
      static_cast<float>(image_size.height) / input_size.height;
  std::vector<Armor> armors;
  struct Candidate {
    cv::Rect box;
    std::array<cv::Point2f, 4> corners;  // 左上、左下、右下、右上
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
  if (params.conf_threshold >= 1.0F) {
    logit_threshold = std::numeric_limits<float>::infinity();
  } else if (params.conf_threshold > 0.0F) {
    logit_threshold =
        std::log(params.conf_threshold / (1.0F - params.conf_threshold));
  }

  for (std::size_t row = 0; row < rows; ++row) {
    const float* line = data + static_cast<size_t>(row) * kOutputCols;
    if (line[kConfCol] < logit_threshold ||
        !std::all_of(line, line + kOutputCols,
                     [](float v) { return std::isfinite(v); })) {
      continue;
    }
    const float confidence = sigmoid(line[kConfCol]);

    // 颜色：0 蓝 1 红 2 灰 3 紫，灰和紫（未激活/混色）直接丢掉。
    const int color_id = argmax(line + kColorCol, 4);
    if (color_id >= 2) {
      continue;
    }
    int color = color_id == 0 ? BLUE : RED;
    if (params.swap_color) {
      color = color == BLUE ? RED : BLUE;
    }
    if (color != params.detect_color) {
      continue;
    }

    Candidate candidate;
    candidate.color = color;
    candidate.genre = argmax(line + kGenreCol, 9);
    candidate.confidence = confidence;
    for (int i = 0; i < 4; ++i) {
      candidate.corners[i] =
          cv::Point2f(line[i * 2] * scale_x, line[i * 2 + 1] * scale_y);
    }
    candidate.box = cv::boundingRect(std::vector<cv::Point2f>(
        candidate.corners.begin(), candidate.corners.end()));

    candidates.push_back(candidate);
    boxes.push_back(candidate.box);
    scores.push_back(candidate.confidence);
  }

  if (!candidates.empty()) {
    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes, scores, params.conf_threshold,
                      params.nms_threshold, kept);
    armors.reserve(kept.size());

    for (int index : kept) {
      if (index < 0 || index >= static_cast<int>(candidates.size())) {
        continue;
      }
      const Candidate& candidate = candidates[index];
      const GenreInfo& genre = kGenres[static_cast<size_t>(candidate.genre)];

      if (std::find(params.ignore_classes.begin(), params.ignore_classes.end(),
                    std::string(genre.number)) != params.ignore_classes.end()) {
        continue;
      }

      Armor armor(makeLight(candidate.corners[0], candidate.corners[1],
                            candidate.color),
                  makeLight(candidate.corners[3], candidate.corners[2],
                            candidate.color));
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

  return armors;
}
void drawNeuralResults(cv::Mat& img, const std::vector<Armor>& armors) {
  for (const auto& armor : armors) {
    const auto& left = armor.left_light;
    const auto& right = armor.right_light;

    // 网络输出角点即最终交给 PnP 的角点。
    cv::line(img, left.top, left.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, right.top, right.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, left.top, right.top, cv::Scalar(0, 255, 0), 2);
    cv::line(img, right.bottom, left.bottom, cv::Scalar(0, 255, 0), 2);

    cv::putText(img, armor.classfication_result, left.top + cv::Point2f(0, -4),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 1);
  }
}
}  // namespace rm_auto_aim
