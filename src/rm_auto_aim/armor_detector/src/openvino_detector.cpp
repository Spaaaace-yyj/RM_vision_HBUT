#include "armor_detector/openvino_detector.hpp"

#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <opencv2/dnn.hpp>
#include <stdexcept>
#include <utility>

#include "armor_detector/neural_postprocessor.hpp"
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/openvino.hpp>
#endif

namespace rm_auto_aim {
namespace {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
using Clock = std::chrono::steady_clock;
float elapsedMs(Clock::time_point start) {
  return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
}
#endif
}  // namespace

struct OpenVINODetector::Impl {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  struct Slot {
    ov::InferRequest request;
    NeuralFrameResult result;
    NeuralDetectorParams params;
    cv::Size image_size;
    Clock::time_point submitted, inference_start;
    std::exception_ptr error;
    bool ready = false;
  };
  ov::Core core;
  ov::CompiledModel compiled;
  cv::Size input_size;
  int stream_count = 0;
  int thread_count = 0;
  std::vector<std::unique_ptr<Slot>> slots;
  // Serialize admission and consumption separately; inference itself runs
  // outside these locks.
  std::mutex producer_mutex, consumer_mutex;
  mutable std::mutex state_mutex;
  std::condition_variable completed;
  std::deque<std::size_t> idle, ordered;

  void finish(std::size_t index, std::exception_ptr error) noexcept {
    // Callback does no decoding, logging, ROS publishing, or blocking user
    // work.
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      auto& slot = *slots[index];
      slot.result.timing.inference_ms = elapsedMs(slot.inference_start);
      slot.error = error;
      slot.ready = true;
    }
    completed.notify_one();
  }

  ~Impl() {
    // The owner must stop/join submit/receive callers before destruction.
    // Do not hold state_mutex here: callbacks need it to finish.
    for (auto& slot : slots) {
      try {
        slot->request.wait();
      } catch (...) {
      }  // Failure is carried by the result.
    }
  }
#endif
};

OpenVINODetector::OpenVINODetector(const std::string& model_path,
                                   OpenVINOConfig config)
    : impl_(new Impl) {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  if (config.threads < 0 || config.streams < 0 || config.requests < 0 ||
      config.requests > 1024) {
    throw std::invalid_argument(
        "OpenVINO threads/streams/requests must be nonnegative; requests <= "
        "1024");
  }
  auto model = impl_->core.read_model(model_path);
  if (model->inputs().size() != 1 || model->outputs().size() != 1 ||
      model->input().get_partial_shape().is_dynamic() ||
      model->output().get_partial_shape().is_dynamic()) {
    throw std::runtime_error(
        "Expected one static NCHW input and one static [1,N,22] output");
  }
  const auto input = model->input().get_shape();
  const auto output = model->output().get_shape();
  if (input.size() != 4 || input[0] != 1 || input[1] != 3 || input[2] == 0 ||
      input[3] == 0 || output.size() != 3 || output[0] != 1 || output[1] == 0 ||
      output[2] != 22) {
    throw std::runtime_error(
        "Model must match RobotPilots: [1,3,H,W] -> [1,N,22], batch=1");
  }
  if (model->input().get_element_type() != ov::element::f16 &&
      model->input().get_element_type() != ov::element::f32) {
    throw std::runtime_error("Model input must be FP16 or FP32");
  }
  impl_->input_size =
      cv::Size(static_cast<int>(input[3]), static_cast<int>(input[2]));
  // Host always writes normalized FP32. Runtime converts to the model's
  // original dtype.
  ov::preprocess::PrePostProcessor processor(model);
  processor.input().tensor().set_element_type(ov::element::f32);
  processor.input().preprocess().convert_element_type(
      model->input().get_element_type());
  processor.output().tensor().set_element_type(ov::element::f32);
  model = processor.build();

  ov::AnyMap properties{{ov::hint::performance_mode.name(),
                         ov::hint::PerformanceMode::THROUGHPUT}};
  if (config.threads) {
    properties[ov::inference_num_threads.name()] = config.threads;
  }
  if (config.streams) {
    properties[ov::num_streams.name()] = ov::streams::Num(config.streams);
  }
  if (config.requests) {
    properties[ov::hint::num_requests.name()] = config.requests;
  }
  impl_->compiled = impl_->core.compile_model(model, "CPU", properties);
  impl_->stream_count =
      static_cast<int>(impl_->compiled.get_property(ov::num_streams));
  impl_->thread_count = impl_->compiled.get_property(ov::inference_num_threads);
  const auto count =
      config.requests
          ? static_cast<std::size_t>(config.requests)
          : impl_->compiled.get_property(ov::optimal_number_of_infer_requests);
  if (count == 0 || count > 1024) {
    throw std::runtime_error("Invalid inference request count");
  }
  impl_->slots.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto slot = std::make_unique<Impl::Slot>();
    slot->request = impl_->compiled.create_infer_request();
    impl_->slots.push_back(std::move(slot));
    // Capture a non-owning Impl pointer, never an InferRequest by value (would
    // form a cycle).
    impl_->slots[i]->request.set_callback(
        [state = impl_.get(), i](std::exception_ptr error) {
          state->finish(i, error);
        });
    impl_->idle.push_back(i);
  }
#else
  (void)model_path;
  (void)config;
  throw std::runtime_error("OpenVINO support was not built; see OPENVINO.md");
#endif
}

OpenVINODetector::~OpenVINODetector() = default;

bool OpenVINODetector::available() {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  return true;
#else
  return false;
#endif
}

bool OpenVINODetector::submit(const cv::Mat& rgb, std::uint64_t frame_id,
                              const NeuralDetectorParams& params) {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  if (rgb.empty() || rgb.type() != CV_8UC3) {
    throw std::invalid_argument(
        "OpenVINODetector expects nonempty RGB CV_8UC3");
  }
  std::lock_guard<std::mutex> producer_lock(impl_->producer_mutex);
  std::size_t index;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    if (impl_->idle.empty()) {
      return false;
    }
    index = impl_->idle.front();
    // Copy params before consuming a slot so allocation failure cannot leave a
    // sequence gap.
    impl_->slots[index]->params = params;
    auto& slot = *impl_->slots[index];
    slot.result = {};
    slot.result.frame_id = frame_id;
    slot.error = nullptr;
    slot.ready = false;
    slot.image_size = rgb.size();
    slot.submitted = Clock::now();
    slot.inference_start = slot.submitted;
    impl_->ordered.push_back(index);
    impl_->idle.pop_front();
  }
  auto& slot = *impl_->slots[index];
  try {
    auto tensor = slot.request.get_input_tensor();
    const int shape[] = {1, 3, impl_->input_size.height,
                         impl_->input_size.width};
    // Use the request's own input storage; no view into a camera/ROS buffer
    // survives submit().
    cv::Mat blob(4, shape, CV_32F, tensor.data<float>());
    cv::dnn::blobFromImage(rgb, blob, 1.0 / 255.0, impl_->input_size,
                           cv::Scalar(), false, false, CV_32F);
    slot.result.timing.preprocess_ms = elapsedMs(slot.submitted);
    slot.inference_start = Clock::now();
    slot.request.start_async();
  } catch (...) {
    impl_->finish(index, std::current_exception());
  }
  return true;
#else
  (void)rgb;
  (void)frame_id;
  (void)params;
  return false;
#endif
}

bool OpenVINODetector::tryReceive(NeuralFrameResult& result) {
  return waitReceive(result, std::chrono::milliseconds(0));
}

bool OpenVINODetector::waitReceive(NeuralFrameResult& result,
                                   std::chrono::milliseconds timeout) {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  std::lock_guard<std::mutex> consumer_lock(impl_->consumer_mutex);
  std::unique_lock<std::mutex> lock(impl_->state_mutex);
  if (!impl_->completed.wait_for(lock, timeout, [this] {
        return !impl_->ordered.empty() &&
               impl_->slots[impl_->ordered.front()]->ready;
      })) {
    return false;
  }
  const auto index = impl_->ordered.front();
  auto& slot = *impl_->slots[index];
  // Slot remains occupied until decoding is complete, protecting output storage
  // from reuse.
  lock.unlock();
  const auto postprocess_start = Clock::now();
  try {
    if (slot.error) {
      std::rethrow_exception(slot.error);
    }
    auto output = slot.request.get_output_tensor();
    slot.result.armors =
        decodeNeuralOutput(output.data<float>(), output.get_shape()[1],
                           impl_->input_size, slot.image_size, slot.params);
  } catch (const std::exception& error) {
    slot.result.error = error.what();
  } catch (...) {
    slot.result.error = "Unknown inference/decode failure";
  }
  slot.result.timing.postprocess_ms = elapsedMs(postprocess_start);
  slot.result.timing.total_ms = slot.result.timing.preprocess_ms +
                                slot.result.timing.inference_ms +
                                slot.result.timing.postprocess_ms;
  slot.result.latency_ms = elapsedMs(slot.submitted);
  result = std::move(slot.result);
  lock.lock();
  impl_->ordered.pop_front();
  impl_->idle.push_back(index);
  return true;
#else
  (void)result;
  (void)timeout;
  return false;
#endif
}

std::size_t OpenVINODetector::pending() const {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->ordered.size();
#else
  return 0;
#endif
}
std::size_t OpenVINODetector::capacity() const {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  return impl_->slots.size();
#else
  return 0;
#endif
}
int OpenVINODetector::streams() const {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  return impl_->stream_count;
#else
  return 0;
#endif
}
int OpenVINODetector::threads() const {
#ifdef ARMOR_DETECTOR_WITH_OPENVINO
  return impl_->thread_count;
#else
  return 0;
#endif
}
}  // namespace rm_auto_aim
