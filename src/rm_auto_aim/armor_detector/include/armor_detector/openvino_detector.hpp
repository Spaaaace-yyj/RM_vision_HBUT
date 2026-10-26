// CPU inference request pool. Public API contains no OpenVINO or ROS types.
#ifndef ARMOR_DETECTOR__OPENVINO_DETECTOR_HPP_
#define ARMOR_DETECTOR__OPENVINO_DETECTOR_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "armor_detector/neural_detector.hpp"

namespace rm_auto_aim {
struct OpenVINOConfig {
  int threads =
      0;  // Whole compiled model's CPU thread budget; 0 = runtime default.
  int streams =
      0;  // Parallel execution streams; 0 = THROUGHPUT auto selection.
  int requests =
      0;  // Bounded request/result capacity; 0 = runtime recommendation.
};

struct NeuralFrameResult {
  std::uint64_t frame_id = 0;
  std::vector<Armor> armors;
  NeuralDetector::Timing timing;
  float latency_ms =
      0;  // submit -> ordered receive; includes waiting behind earlier frames.
  std::string
      error;  // An accepted failed frame still produces one ordered result.
};

class OpenVINODetector {
 public:
  explicit OpenVINODetector(const std::string &model_path,
                            OpenVINOConfig config = {});
  // Stop accepting and wait for all callbacks before destroying their state.
  ~OpenVINODetector();
  OpenVINODetector(const OpenVINODetector &) = delete;
  OpenVINODetector &operator=(const OpenVINODetector &) = delete;

  // RGB CV_8UC3. Copies/preprocesses pixels before returning; no camera-buffer
  // lifetime dependency. false = full, nothing accepted. No unbounded pending
  // queue and no silent frame replacement. Frame IDs need not be consecutive;
  // receive order is the actual submission order.
  bool submit(const cv::Mat &rgb, std::uint64_t frame_id,
              const NeuralDetectorParams &params);
  bool tryReceive(NeuralFrameResult &result);
  bool waitReceive(NeuralFrameResult &result,
                   std::chrono::milliseconds timeout);
  std::size_t pending() const;
  std::size_t capacity() const;
  int streams() const;
  int threads() const;
  static bool available();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace rm_auto_aim
#endif
