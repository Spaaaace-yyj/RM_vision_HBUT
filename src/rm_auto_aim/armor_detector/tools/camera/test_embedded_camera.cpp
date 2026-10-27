#include "armor_detector/embedded_camera.hpp"
#include <CameraApi.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) { throw std::runtime_error(message); }
}
template<typename F> void requireThrow(F fn, const char * message)
{
    bool threw = false;
    try { fn(); } catch (const std::runtime_error &) { threw = true; }
    require(threw, message);
}
struct FakeSdk
{
    int devices = 1;
    int init_status = 0, exposure_status = 0, read_status = 0, process_status = 0, flip_status = 0;
    int width = 2, height = 2;
    int closed = 0, released = 0, acquired = 0;
    int analog = -1, r = 10, g = 20, b = 30, gamma = -1, saturation = -1;
    double exposure = 0;
    BYTE pixels[12]{};
    std::vector<std::string> events;
} sdk;
}  // namespace

// These signatures are checked against the repository's actual CameraApi.h.
extern "C"
{
CameraSdkStatus CameraSdkInit(int) { return 0; }
CameraSdkStatus CameraEnumerateDevice(tSdkCameraDevInfo *, int * counts)
{
    *counts = sdk.devices;
    return 0;
}
CameraSdkStatus CameraInit(tSdkCameraDevInfo *, int, int, CameraHandle * handle)
{
    *handle = 7;
    return sdk.init_status;
}
CameraSdkStatus CameraUnInit(CameraHandle handle)
{
    require(handle == 7, "wrong handle closed");
    ++sdk.closed;
    return 0;
}
CameraSdkStatus CameraSetAeState(CameraHandle, BOOL automatic)
{
    require(!automatic, "manual exposure required");
    return 0;
}
CameraSdkStatus CameraSetExposureTime(CameraHandle, double exposure)
{
    sdk.exposure = exposure;
    return sdk.exposure_status;
}
CameraSdkStatus CameraSetAnalogGain(CameraHandle, int gain) { sdk.analog = gain; return 0; }
CameraSdkStatus CameraGetGain(CameraHandle, int * r, int * g, int * b)
{
    *r = sdk.r; *g = sdk.g; *b = sdk.b;
    return 0;
}
CameraSdkStatus CameraSetGain(CameraHandle, int r, int g, int b)
{
    sdk.r = r; sdk.g = g; sdk.b = b;
    return 0;
}
CameraSdkStatus CameraSetGamma(CameraHandle, int gamma) { sdk.gamma = gamma; return 0; }
CameraSdkStatus CameraSetSaturation(CameraHandle, int value) { sdk.saturation = value; return 0; }
CameraSdkStatus CameraSetIspOutFormat(CameraHandle, UINT format)
{
    require(format == CAMERA_MEDIA_TYPE_RGB8, "RGB8 output required");
    return 0;
}
CameraSdkStatus CameraPlay(CameraHandle) { return 0; }
CameraSdkStatus CameraGetImageBuffer(CameraHandle, tSdkFrameHead * frame, BYTE ** data, UINT timeout)
{
    require(timeout == 1000, "unbounded SDK wait");
    if (sdk.read_status != 0) { return sdk.read_status; }
    require(sdk.acquired == sdk.released, "previous raw buffer still held");
    ++sdk.acquired;
    sdk.events.emplace_back("acquire");
    frame->iWidth = sdk.width;
    frame->iHeight = sdk.height;
    *data = sdk.pixels;
    return 0;
}
CameraSdkStatus CameraReleaseImageBuffer(CameraHandle, BYTE * data)
{
    require(data == sdk.pixels, "wrong SDK buffer released");
    ++sdk.released;
    sdk.events.emplace_back("release");
    std::fill(std::begin(sdk.pixels), std::end(sdk.pixels), BYTE{99});
    return 0;
}
CameraSdkStatus CameraImageProcess(CameraHandle, BYTE * raw, BYTE * out, tSdkFrameHead * frame)
{
    sdk.events.emplace_back("process");
    if (sdk.process_status != 0) { return sdk.process_status; }
    std::copy(raw, raw + frame->iWidth * frame->iHeight * 3, out);
    return 0;
}
CameraSdkStatus CameraFlipFrameBuffer(BYTE * data, tSdkFrameHead * frame, int flags)
{
    require(flags == 3, "wrong flip flags");
    if (sdk.flip_status != 0) { return sdk.flip_status; }
    std::reverse(data, data + frame->iWidth * frame->iHeight * 3);
    return 0;
}
}  // extern "C"

int main()
{
    using rm_auto_aim::EmbeddedCamera;
    using rm_auto_aim::EmbeddedCameraParams;
    try
    {
        sdk = {};
        {
            EmbeddedCameraParams params;
            params.exposure_time = 4000;
            params.analog_gain = 50;
            params.r_gain = 155;  // unspecified G/B must preserve SDK values
            params.gamma = 100;
            params.saturation = 120;
            params.flip_image = true;
            EmbeddedCamera camera(params);
            require(sdk.exposure == 4000 && sdk.analog == 50, "camera parameters lost");
            require(sdk.r == 155 && sdk.g == 20 && sdk.b == 30, "partial RGB gain update failed");
            require(sdk.gamma == 100 && sdk.saturation == 120, "ISP parameters lost");
            std::vector<std::uint8_t> first, second;
            std::uint32_t width = 0, height = 0;
            for (int i = 0; i < 12; ++i) { sdk.pixels[i] = i; }
            require(camera.read(first, width, height, [] { sdk.events.emplace_back("stamp"); }), "read failed");
            require(width == 2 && height == 2 && first.size() == 12, "wrong frame shape");
            require(first.front() == 11 && first.back() == 0, "flip/output mismatch");
            require(sdk.events == std::vector<std::string>{"acquire", "stamp", "process", "release"},
                "timestamp or SDK release order wrong");
            require(camera.read(second, width, height), "second read failed");
            require(second.front() == 99 && first.front() == 11, "previous frame overwritten");

            sdk.read_status = CAMERA_STATUS_TIME_OUT;
            require(!camera.read(second, width, height), "timeout not reported");
            require(sdk.released == 2, "released buffer on failed acquisition");
            sdk.read_status = -1;
            requireThrow([&] { camera.read(second, width, height); }, "SDK read error swallowed");
            sdk.read_status = 0;
            sdk.process_status = -1;
            requireThrow([&] { camera.read(second, width, height); }, "ISP error swallowed");
            require(sdk.released == 3, "raw buffer leaked on ISP error");
            sdk.process_status = 0;
            sdk.flip_status = -1;
            requireThrow([&] { camera.read(second, width, height); }, "flip error swallowed");
            require(sdk.released == 4, "raw buffer leaked on flip error");
            sdk.flip_status = 0;
            sdk.width = 0;
            requireThrow([&] { camera.read(second, width, height); }, "invalid dimensions accepted");
            require(sdk.released == 5, "raw buffer leaked on invalid frame");
            sdk.width = 2;
            requireThrow([&] { camera.read(second, width, height, [] { throw std::runtime_error("clock"); }); },
                "callback exception swallowed");
            require(sdk.released == 6, "raw buffer leaked on timestamp exception");
        }
        require(sdk.closed == 1, "camera not closed once");
        sdk = {};
        sdk.exposure_status = -1;
        requireThrow([] { EmbeddedCamera camera({}); }, "setup failure swallowed");
        require(sdk.closed == 1, "camera leaked on setup failure");
        sdk = {};
        sdk.init_status = -1;
        requireThrow([] { EmbeddedCamera camera({}); }, "init failure swallowed");
        require(sdk.closed == 0, "closed uninitialized handle");
        sdk = {};
        sdk.devices = 0;
        requireThrow([] { EmbeddedCamera camera({}); }, "missing device accepted");
        require(sdk.closed == 0, "closed nonexistent camera");
        std::cout << "embedded_camera: passed (configuration, ownership, timestamps, failure cleanup)\n";
        return 0;
    }
    catch (const std::exception & error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
