#include "armor_detector/embedded_camera.hpp"

#include <limits>
#include <stdexcept>
#include <string>

#ifdef ARMOR_DETECTOR_WITH_CAMERA
#include <CameraApi.h>
#endif

namespace rm_auto_aim
{
#ifdef ARMOR_DETECTOR_WITH_CAMERA
namespace
{
void checkCamera(int status, const char * operation)
{
    if (status != CAMERA_STATUS_SUCCESS)
    {
        throw std::runtime_error(std::string(operation) + " failed, status=" + std::to_string(status));
    }
}

struct SdkFrameBuffer
{
    CameraHandle camera;
    BYTE * data;
    ~SdkFrameBuffer() { CameraReleaseImageBuffer(camera, data); }
};
}  // namespace

struct EmbeddedCamera::Impl
{
    CameraHandle handle = -1;
    bool flip = false;
    ~Impl()
    {
        if (handle >= 0) { CameraUnInit(handle); }
    }
};

EmbeddedCamera::EmbeddedCamera(const EmbeddedCameraParams & params) : impl_(std::make_unique<Impl>())
{
    checkCamera(CameraSdkInit(1), "CameraSdkInit");
    int counts = 1;
    tSdkCameraDevInfo device{};
    checkCamera(CameraEnumerateDevice(&device, &counts), "CameraEnumerateDevice");
    if (counts == 0) { throw std::runtime_error("use_embedded_camera=true but no MindVision camera found"); }
    CameraHandle handle = -1;
    checkCamera(CameraInit(&device, -1, -1, &handle), "CameraInit");
    impl_->handle = handle;  // RAII also closes the handle if subsequent setup fails.
    checkCamera(CameraSetAeState(handle, false), "CameraSetAeState");
    checkCamera(CameraSetExposureTime(handle, params.exposure_time), "CameraSetExposureTime");
    if (params.analog_gain >= 0)
    {
        checkCamera(CameraSetAnalogGain(handle, params.analog_gain), "CameraSetAnalogGain");
    }
    if (params.r_gain >= 0 || params.g_gain >= 0 || params.b_gain >= 0)
    {
        int r = 0, g = 0, b = 0;
        checkCamera(CameraGetGain(handle, &r, &g, &b), "CameraGetGain");
        checkCamera(CameraSetGain(handle,
            params.r_gain < 0 ? r : params.r_gain,
            params.g_gain < 0 ? g : params.g_gain,
            params.b_gain < 0 ? b : params.b_gain), "CameraSetGain");
    }
    if (params.gamma >= 0) { checkCamera(CameraSetGamma(handle, params.gamma), "CameraSetGamma"); }
    if (params.saturation >= 0)
    {
        checkCamera(CameraSetSaturation(handle, params.saturation), "CameraSetSaturation");
    }
    impl_->flip = params.flip_image;
    checkCamera(CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_RGB8), "CameraSetIspOutFormat");
    checkCamera(CameraPlay(handle), "CameraPlay");
}

EmbeddedCamera::~EmbeddedCamera() = default;

bool EmbeddedCamera::read(
    std::vector<std::uint8_t> & rgb, std::uint32_t & width, std::uint32_t & height,
    const std::function<void()> & on_acquired)
{
    tSdkFrameHead frame{};
    BYTE * raw = nullptr;
    const int status = CameraGetImageBuffer(impl_->handle, &frame, &raw, 1000);
    if (status == CAMERA_STATUS_TIME_OUT) { return false; }
    checkCamera(status, "CameraGetImageBuffer");
    const SdkFrameBuffer buffer{impl_->handle, raw};
    if (on_acquired) { on_acquired(); }
    if (frame.iWidth <= 0 || frame.iHeight <= 0 ||
        static_cast<std::size_t>(frame.iWidth) >
            std::numeric_limits<std::size_t>::max() / 3 / static_cast<std::size_t>(frame.iHeight))
    {
        throw std::runtime_error("Invalid MindVision frame dimensions");
    }
    rgb.resize(static_cast<std::size_t>(frame.iWidth) * frame.iHeight * 3);
    checkCamera(CameraImageProcess(impl_->handle, raw, rgb.data(), &frame), "CameraImageProcess");
    if (impl_->flip) { checkCamera(CameraFlipFrameBuffer(rgb.data(), &frame, 3), "CameraFlipFrameBuffer"); }
    width = frame.iWidth;
    height = frame.iHeight;
    return true;  // SDK buffer released before detection starts, including on every exception path.
}
#else
struct EmbeddedCamera::Impl {};
EmbeddedCamera::EmbeddedCamera(const EmbeddedCameraParams &)
{
    throw std::runtime_error("MindVision SDK unavailable at build time; use_embedded_camera requires ARMOR_ENABLE_CAMERA=ON and MVSDK_ROOT");
}
EmbeddedCamera::~EmbeddedCamera() = default;
bool EmbeddedCamera::read(
    std::vector<std::uint8_t> &, std::uint32_t &, std::uint32_t &, const std::function<void()> &)
{
    return false;
}
#endif
}  // namespace rm_auto_aim
