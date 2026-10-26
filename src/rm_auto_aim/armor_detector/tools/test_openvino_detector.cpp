// Standalone CPU feasibility test. Measures the same preprocessing/decoder used
// by the ROS node.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <thread>

#include "armor_detector/openvino_detector.hpp"

namespace {
using Clock = std::chrono::steady_clock;
double percentile(std::vector<float> values, double fraction) {
  if (values.empty()) {
    return 0;
  }
  const auto i = static_cast<std::size_t>((values.size() - 1) * fraction);
  std::nth_element(values.begin(), values.begin() + i, values.end());
  return values[i];
}
}  // namespace
int main(int argc, char** argv) {
  const char* keys =
      "{help h||Show usage}"
      "{model||Required: RobotPilots ONNX or OpenVINO XML model}"
      "{input||Image/video filename or camera:0; empty = synthetic RGB "
      "1280x1024}"
      "{frames|900|Number of measured input frames}"
      "{warmup|30|Warmup frames, excluded from all measured statistics}"
      "{fps|0|Input pacing: 0 = maximum throughput; 150 = simulate 150 FPS "
      "arrivals}"
      "{threads|0|Whole-model CPU thread budget, 0 = automatic}"
      "{streams|0|Parallel inference streams, 0 = automatic}"
      "{requests|0|Bounded request/result capacity, 0 = automatic}"
      "{color|0|0 blue, 1 red}"
      "{conf|0.65|Confidence threshold}"
      "{nms|0.45|NMS threshold}"
      "{swap_color|false|Swap model red/blue mapping}"
      "{csv||Optional per-frame timing output file}";
  cv::CommandLineParser parser(argc, argv, keys);
  if (parser.has("help") || argc == 1) {
    parser.printMessage();
    return 0;
  }
  try {
    const std::string model = parser.get<std::string>("model");
    const auto source = parser.get<std::string>("input");
    const int frames = parser.get<int>("frames"),
              warmup = parser.get<int>("warmup");
    const double fps = parser.get<double>("fps");
    rm_auto_aim::OpenVINOConfig config;
    config.threads = parser.get<int>("threads");
    config.streams = parser.get<int>("streams");
    config.requests = parser.get<int>("requests");
    rm_auto_aim::NeuralDetectorParams params;
    params.detect_color = parser.get<int>("color");
    params.conf_threshold = parser.get<float>("conf");
    params.nms_threshold = parser.get<float>("nms");
    params.swap_color = parser.get<bool>("swap_color");
    const auto csv_path = parser.get<std::string>("csv");
    if (!parser.check()) {
      parser.printErrors();
      return 1;
    }
    if (model.empty() || frames <= 0 || warmup < 0 || !std::isfinite(fps) ||
        fps < 0 || (params.detect_color != 0 && params.detect_color != 1)) {
      throw std::invalid_argument(
          "Set --model; frames > 0, warmup >= 0, fps >= 0, color = 0/1");
    }
    if (!rm_auto_aim::OpenVINODetector::available()) {
      throw std::runtime_error(
          "OpenVINO backend was not built; see OPENVINO.md");
    }
    cv::Mat fixed_rgb;
    cv::VideoCapture capture;
    if (source.empty()) {
      fixed_rgb = cv::Mat(1024, 1280, CV_8UC3, cv::Scalar(32, 32, 32));
    } else if (source.rfind("camera:", 0) == 0) {
      capture.open(std::stoi(source.substr(7)));
      if (!capture.isOpened()) {
        throw std::runtime_error("Cannot open camera: " + source);
      }
    } else {
      const cv::Mat image = cv::imread(source);
      if (!image.empty()) {
        cv::cvtColor(image, fixed_rgb, cv::COLOR_BGR2RGB);
      } else if (!capture.open(source)) {
        throw std::runtime_error("Cannot open image/video: " + source);
      }
    }
    const auto read = [&](cv::Mat& rgb) {
      if (!fixed_rgb.empty()) {
        rgb = fixed_rgb;
        return true;
      }
      cv::Mat bgr;
      if (!capture.read(bgr) || bgr.empty()) {
        return false;
      }
      cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
      return true;
    };
    // Avoid an unrelated OpenCV worker pool competing with OpenVINO's CPU
    // workers.
    cv::setNumThreads(1);
    rm_auto_aim::OpenVINODetector detector(model, config);
    std::cout << "CPU config: threads=" << detector.threads()
              << " streams=" << detector.streams()
              << " requests=" << detector.capacity() << "\nSource: "
              << (source.empty()
                      ? "synthetic (performance only, not an accuracy test)"
                      : source)
              << "\n";
    std::deque<std::uint64_t> expected;
    std::uint64_t next_id = 0;
    std::ofstream csv;
    if (!csv_path.empty()) {
      csv.open(csv_path);
      if (!csv) {
        throw std::runtime_error("Cannot open CSV: " + csv_path);
      }
      csv << "frame_id,latency_ms,pre_ms,infer_ms,post_ms,compute_ms,armors,"
             "error\n";
    }
    std::size_t completed = 0, errors = 0, armor_count = 0;
    std::vector<float> latencies, inference;
    double pre_ms = 0, post_ms = 0;
    const auto receive = [&](bool blocking, bool measured) {
      rm_auto_aim::NeuralFrameResult result;
      const bool ready =
          blocking ? detector.waitReceive(result, std::chrono::seconds(30))
                   : detector.tryReceive(result);
      if (!ready) {
        if (blocking) {
          throw std::runtime_error("Inference timed out after 30 seconds");
        }
        return false;
      }
      if (expected.empty() || result.frame_id != expected.front()) {
        throw std::runtime_error(
            "Out-of-order, missing or duplicate frame result");
      }
      expected.pop_front();
      if (!result.error.empty()) {
        if (!measured) {
          throw std::runtime_error("Warmup failed: " + result.error);
        }
        ++errors;
        if (errors <= 3) {
          std::cerr << "Frame " << result.frame_id << ": " << result.error
                    << '\n';
        }
      }
      if (measured) {
        ++completed;
        armor_count += result.armors.size();
        latencies.push_back(result.latency_ms);
        inference.push_back(result.timing.inference_ms);
        pre_ms += result.timing.preprocess_ms;
        post_ms += result.timing.postprocess_ms;
        if (csv) {
          csv << result.frame_id << ',' << result.latency_ms << ','
              << result.timing.preprocess_ms << ','
              << result.timing.inference_ms << ','
              << result.timing.postprocess_ms << ',' << result.timing.total_ms
              << ',' << result.armors.size() << ','
              << (result.error.empty() ? "ok" : "failed") << '\n';
        }
      }
      return true;
    };
    for (int i = 0; i < warmup; ++i) {
      cv::Mat rgb;
      if (!read(rgb)) {
        throw std::runtime_error("Source ended during warmup; reduce --warmup");
      }
      const auto id = next_id++;
      while (!detector.submit(rgb, id, params)) {
        receive(true, false);
      }
      expected.push_back(id);
      while (receive(false, false)) {
      }
    }
    while (detector.pending()) {
      receive(true, false);
    }

    std::size_t input_count = 0, accepted = 0, rejected = 0, late_inputs = 0,
                peak_pending = 0;
    const auto start = Clock::now();
    for (int i = 0; i < frames; ++i) {
      if (fps > 0) {
        const auto due = start + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(i / fps));
        std::this_thread::sleep_until(due);
        if (Clock::now() - due > std::chrono::duration<double>(1.0 / fps)) {
          ++late_inputs;
        }
      }
      while (receive(false, true)) {
      }
      cv::Mat rgb;
      if (!read(rgb)) {
        break;
      }
      ++input_count;
      const auto id = next_id++;
      bool submitted = detector.submit(rgb, id, params);
      // Saturation mode applies backpressure. Paced mode counts full-pool
      // arrivals as rejected.
      while (!submitted && fps == 0) {
        receive(true, true);
        submitted = detector.submit(rgb, id, params);
      }
      if (submitted) {
        ++accepted;
        expected.push_back(id);
      } else {
        ++rejected;
      }
      peak_pending = std::max(peak_pending, detector.pending());
    }
    const double input_seconds =
        std::chrono::duration<double>(Clock::now() - start).count();
    while (detector.pending()) {
      receive(true, true);
    }
    const double seconds =
        std::chrono::duration<double>(Clock::now() - start).count();
    if (completed != accepted || !expected.empty() || input_count == 0) {
      throw std::runtime_error("Result count mismatch or empty source");
    }
    std::cout << std::fixed << std::setprecision(2) << "inputs=" << input_count
              << " accepted=" << accepted << " completed=" << completed
              << " rejected=" << rejected << " failed=" << errors
              << " late_inputs=" << late_inputs
              << " peak_pending=" << peak_pending << '/' << detector.capacity()
              << '\n'
              << "source FPS=" << input_count / input_seconds
              << " completed FPS (including final drain)="
              << completed / seconds << '\n'
              << "submit->ordered-receive latency p50="
              << percentile(latencies, .50)
              << " p95=" << percentile(latencies, .95)
              << " p99=" << percentile(latencies, .99)
              << " ms; inference p50=" << percentile(inference, .50) << " ms\n"
              << "average pre=" << (completed ? pre_ms / completed : 0)
              << " post=" << (completed ? post_ms / completed : 0)
              << " ms; detected armors=" << armor_count << '\n';
    const bool complete_source =
        input_count == static_cast<std::size_t>(frames);
    if (!complete_source) {
      std::cerr << "Source ended before --frames was reached\n";
    }
    if (fps > 0) {
      std::cout
          << "Paced test: "
          << ((rejected == 0 && errors == 0 && late_inputs == 0 &&
               complete_source)
                  ? "PASS"
                  : "FAIL")
          << " (this source and duration only; ROS/PnP/display are excluded)\n";
    }
    return errors || rejected || !complete_source || (fps > 0 && late_inputs)
               ? 2
               : 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
