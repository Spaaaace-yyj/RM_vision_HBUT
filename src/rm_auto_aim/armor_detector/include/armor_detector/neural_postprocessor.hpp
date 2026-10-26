// Stateless decoding shared by ONNX Runtime and OpenVINO. No ROS dependency.
#ifndef ARMOR_DETECTOR__NEURAL_POSTPROCESSOR_HPP_
#define ARMOR_DETECTOR__NEURAL_POSTPROCESSOR_HPP_
#include <cstddef>

#include "armor_detector/neural_detector.hpp"
namespace rm_auto_aim {
std::vector<Armor> decodeNeuralOutput(const float* data, std::size_t rows,
                                      cv::Size input_size, cv::Size image_size,
                                      const NeuralDetectorParams& params);
void drawNeuralResults(cv::Mat& image, const std::vector<Armor>& armors);
}  // namespace rm_auto_aim
#endif
