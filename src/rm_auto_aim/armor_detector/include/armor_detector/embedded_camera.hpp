#ifndef ARMOR_DETECTOR__EMBEDDED_CAMERA_HPP_
#define ARMOR_DETECTOR__EMBEDDED_CAMERA_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace rm_auto_aim
{
// Same parameter names/units as mindvision_camera; -1 preserves the camera's current value.
struct EmbeddedCameraParams
{
    int exposure_time = 5000;  // microseconds
    int analog_gain = -1;
    int r_gain = -1;
    int g_gain = -1;
    int b_gain = -1;
    int gamma = -1;
    int saturation = -1;
    bool flip_image = false;
};

// Owns the SDK handle. No ROS/OpenCV dependency and no SDK types in the public header.
// Calls must not run concurrently; read() waits at most 1000 ms for a frame from the SDK.
class EmbeddedCamera
{
public:
    explicit EmbeddedCamera(const EmbeddedCameraParams & params);
    ~EmbeddedCamera();
    EmbeddedCamera(const EmbeddedCamera &) = delete;
    EmbeddedCamera & operator=(const EmbeddedCamera &) = delete;

    // Writes RGB8 into caller-owned storage. on_acquired timestamps the frame immediately
    // after CameraGetImageBuffer, before ISP conversion. Timeout returns false; other errors throw.
    bool read(
        std::vector<std::uint8_t> & rgb, std::uint32_t & width, std::uint32_t & height,
        const std::function<void()> & on_acquired = {});

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__EMBEDDED_CAMERA_HPP_
