#include "armor_detector/embedded_camera.hpp"
#include <iostream>
#include <stdexcept>
#include <string>

int main()
{
    try { rm_auto_aim::EmbeddedCamera camera({}); }
    catch (const std::runtime_error & error)
    {
        if (std::string(error.what()).find("MindVision SDK unavailable") != std::string::npos)
        {
            std::cout << "camera_disabled: passed\n";
            return 0;
        }
    }
    return 1;
}
