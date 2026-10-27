# One message per finished detection cycle, including zero detections and failed cycles.
# Pool-full inputs and mode-switch discards have no cycle result; see DebugDetectorFps.
# All durations are milliseconds. Stage durations use steady_clock.
std_msgs/Header header                    # Original image header, unchanged
uint64 frame_id                            # Detector input sequence; gaps indicate rejected/discarded inputs
builtin_interfaces/Time output_stamp       # Sampled immediately after /detector/armors publish() returns
builtin_interfaces/Time finished_stamp     # Sampled after markers/debug drawing, before timing publication
string detector_mode                      # Requested mode: traditional / neural
string neural_backend                     # Requested backend; empty in traditional mode
bool embedded_camera
bool completed                            # Detection/PnP cycle finished without an uncaught stage error
bool result_published                     # False when calibration is unavailable or the cycle failed
bool traditional_fallback
string error                              # Neural failure may still yield a completed traditional fallback
uint32 detected_armors
uint32 published_armors
uint32 pnp_count

float64 camera_wait_ms                     # Embedded only: CameraGetImageBuffer wait until raw frame acquired
float64 camera_process_ms                  # Embedded only: raw acquisition -> RGB conversion/release complete
float64 camera_info_publish_ms             # Embedded only: camera_info publish() call
float64 input_age_ms                       # Image ROS stamp -> image callback entry; includes upstream latency
float64 mutex_wait_ms                      # Callback waiting for detector state lock
float64 drain_previous_ms                  # Time spent completing earlier async frames before this submission
float64 cv_bridge_ms
float64 parameter_update_ms
float64 model_init_ms                      # Lazy backend initialization/check; first-frame compilation included
float64 record_ms

float64 traditional_preprocess_ms          # Grayscale + threshold
float64 traditional_lights_ms
float64 traditional_match_ms
float64 traditional_extract_ms             # Number-image extraction
float64 traditional_classify_ms
float64 traditional_total_ms

float64 preprocess_ms                      # Neural input preparation
float64 inference_ms                       # ORT Run or OpenVINO start_async -> completion callback
float64 postprocess_ms                     # Neural output decoding/NMS
float64 neural_total_ms                    # pre + infer + post; excludes result_wait_ms
float64 neural_latency_ms                  # OpenVINO submit -> ordered receive/decoding complete
float64 result_wait_ms                     # OpenVINO completion callback -> ordered decoding starts
float64 pnp_ms
float64 result_publish_ms                  # /detector/armors publish() call, not subscriber delivery
float64 marker_publish_ms
float64 debug_ms                           # Debug drawing + debug topic publication; zero when disabled
float64 core_ms                            # After draining older results -> own result publish return (or failure)
float64 callback_to_output_ms              # Callback entry -> result publish return; NaN if no output
float64 frame_ready_to_output_ms           # Embedded RGB conversion complete -> output; NaN in topic mode/no output
float64 e2e_ms                             # Original image ROS stamp -> output; NaN if no output
float64 cycle_ms                           # Callback entry -> finished_stamp, including markers/debug work
