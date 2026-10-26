#include <atomic>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#include "armor_detector/neural_postprocessor.hpp"
#include "armor_detector/openvino_detector.hpp"

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}
}  // namespace
int main(int argc, char **argv) {
  try {
    check(argc == 3, "Expected fixture.xml and fixture_bad.xml");
    rm_auto_aim::OpenVINOConfig config;
    config.requests = 4;
    config.streams = 2;
    config.threads = 4;
    rm_auto_aim::OpenVINODetector detector(argv[1], config);
    check(detector.capacity() == 4, "Requested capacity not applied");
    check(detector.streams() == 2, "Requested streams not applied");
    rm_auto_aim::NeuralDetectorParams params;
    params.detect_color = rm_auto_aim::BLUE;
    cv::Mat image(8, 8, CV_8UC3);
    struct Expected {
      std::uint64_t id;
      float x;
      bool empty;
      bool failed;
    };
    std::deque<Expected> expected;
    std::size_t completed = 0;
    const auto receive = [&] {
      rm_auto_aim::NeuralFrameResult result;
      check(detector.waitReceive(result, std::chrono::seconds(10)),
            "Receive timed out");
      const auto value = expected.front();
      expected.pop_front();
      check(result.frame_id == value.id, "Submission order/ID mismatch");
      check(!result.error.empty() == value.failed,
            "Failed frame was lost or not reported");
      if (!value.failed) {
        check(result.armors.empty() == value.empty,
              "Per-frame parameter snapshot mismatch");
        if (!value.empty) {
          check(result.armors.size() == 1, "Wrong detection count");
          const auto &armor = result.armors.front();
          check(armor.number == "3" &&
                    armor.left_light.color == rm_auto_aim::BLUE,
                "Class/color mapping changed");
          check(std::abs(armor.left_light.top.x - value.x) < 0.02F,
                "Input/output storage overwritten by another request");
        }
      }
      check(std::isfinite(result.latency_ms), "Invalid timing");
      ++completed;
    };
    // Nonconsecutive IDs, per-frame params and immediately overwritten caller
    // input storage.
    for (int i = 0; i < 80; ++i) {
      if (detector.pending() == detector.capacity()) {
        receive();
      }
      const int pixel = (i * 17) % 240;
      image.setTo(cv::Scalar::all(pixel));
      params.detect_color = i % 2 ? rm_auto_aim::RED : rm_auto_aim::BLUE;
      params.conf_threshold =
          i == 19 ? -1.0F : 0.65F;  // Accepted decode failure must not stall.
      const std::uint64_t id = 7 + i * 3;
      check(detector.submit(image, id, params), "Unexpected full pool");
      expected.push_back({id, pixel / 255.0F * 4, i % 2 != 0, i == 19});
      image.setTo(cv::Scalar::all(255));
      params.detect_color = rm_auto_aim::RED;
      if (detector.pending() == detector.capacity()) {
        check(!detector.submit(image, 99999, params),
              "Pool/result storage is unbounded");
      }
    }
    while (!expected.empty()) {
      receive();
    }
    check(completed == 80 && detector.pending() == 0, "Lost/duplicate results");
    rm_auto_aim::NeuralFrameResult result;
    check(!detector.tryReceive(result), "Unexpected extra result");
    bool rejected = false;
    try {
      detector.submit(cv::Mat(), 0, params);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "Invalid image was not rejected");
    rejected = false;
    try {
      rm_auto_aim::OpenVINODetector bad(argv[2], config);
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "Incompatible model was not rejected");
    // Real producer and consumer threads exercise admission/receive state
    // concurrently.
    std::atomic<bool> stop{false};
    std::exception_ptr consumer_error;
    std::thread consumer([&] {
      try {
        for (int i = 0; i < 40 && !stop; ++i) {
          rm_auto_aim::NeuralFrameResult frame;
          check(detector.waitReceive(frame, std::chrono::seconds(10)),
                "Concurrent receive timeout");
          check(frame.frame_id == static_cast<std::uint64_t>(10000 + i * 2),
                "Concurrent order mismatch");
          check(frame.error.empty() && frame.armors.size() == 1,
                "Concurrent result missing");
          check(std::abs(frame.armors.front().left_light.top.x -
                         (i * 5) / 255.0F * 4) < .02F,
                "Concurrent input/output corruption");
        }
      } catch (...) {
        consumer_error = std::current_exception();
        stop = true;
      }
    });
    std::exception_ptr producer_error;
    try {
      params.detect_color = rm_auto_aim::BLUE;
      params.conf_threshold = .65F;
      for (int i = 0; i < 40 && !stop; ++i) {
        image.setTo(cv::Scalar::all(i * 5));
        while (!stop && !detector.submit(image, 10000 + i * 2, params)) {
          std::this_thread::yield();
        }
        image.setTo(cv::Scalar::all(255));
      }
    } catch (...) {
      producer_error = std::current_exception();
      stop = true;
    }
    consumer.join();
    if (producer_error) {
      std::rethrow_exception(producer_error);
    }
    if (consumer_error) {
      std::rethrow_exception(consumer_error);
    }
    // Destruction must join callbacks, even without consuming pending results.
    for (int repeat = 0; repeat < 3; ++repeat) {
      rm_auto_aim::OpenVINODetector pending(argv[1], config);
      params.conf_threshold = 0.65F;
      for (int i = 0; i < 4; ++i) {
        pending.submit(image, i, params);
      }
    }
    std::cout << "PASS: bounded pool, ordered IDs, independent buffers, "
                 "parameter snapshots, "
                 "error advancement, model validation and pending shutdown\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
