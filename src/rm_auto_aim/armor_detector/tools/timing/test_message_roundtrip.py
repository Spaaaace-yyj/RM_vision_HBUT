"""Run after building/sourcing auto_aim_interfaces; no ROS nodes or camera needed."""
import math

from rclpy.serialization import deserialize_message, serialize_message
from auto_aim_interfaces.msg import DebugDetectorFps, DebugDetectorTiming


def main():
    timing = DebugDetectorTiming()
    timing.header.stamp.sec = 42
    timing.header.stamp.nanosec = 123456789
    timing.header.frame_id = "camera_optical_frame"
    timing.frame_id = 2**53 + 17
    timing.detector_mode = "neural"
    timing.neural_backend = "openvino"
    timing.completed = True
    timing.result_published = False
    timing.traditional_fallback = True
    timing.error = "backend failed; traditional fallback"
    timing.e2e_ms = math.nan
    timing.frame_ready_to_output_ms = math.nan
    timing.inference_ms = 9.8
    timing.result_wait_ms = 1.25
    received = deserialize_message(serialize_message(timing), DebugDetectorTiming)
    assert received.header == timing.header and received.frame_id == timing.frame_id
    assert received.completed and received.traditional_fallback and not received.result_published
    assert received.error == timing.error and math.isnan(received.e2e_ms)
    assert math.isnan(received.frame_ready_to_output_ms)
    assert received.inference_ms == 9.8 and received.result_wait_ms == 1.25

    fps = DebugDetectorFps()
    fps.window_seconds = 2.015
    fps.input_fps = 103.0
    fps.rejected_fps = 104 / fps.window_seconds
    fps.completed_total = 2**53 + 99
    fps.pending = 1
    fps.capacity = 1
    received = deserialize_message(serialize_message(fps), DebugDetectorFps)
    assert received == fps
    print("Detector messages: timestamp/frame ID, NaN, fallback and FPS serialization passed")


if __name__ == "__main__":
    main()
