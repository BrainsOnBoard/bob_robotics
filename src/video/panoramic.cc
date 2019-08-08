// BoB robotics includes
#include "common/logging.h"
#include "video/opencvinput.h"
#include "video/panoramic.h"
#ifndef _WIN32
#include "os/video.h"
#include "video/see3cam_cu40.h"
#endif

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Video {

//! Try to find a panoramic camera on the current machine and return it
std::unique_ptr<Input> getPanoramicCamera()
{
#ifdef _WIN32
    // for Windows we currently just select the first camera
    return std::make_unique<OpenCVInput>(0, cv::Size(1280, 720), "webcam360");
#else
    // get vector of video input devices on system
    auto cameras = OS::Video::getCameras();

    // list of preferred cameras in priority order
    static const std::array<std::string, 3> prefCameras{
        See3CamDeviceName, PixProUSBDeviceName, Webcam360DeviceName
    };

    int deviceNum = -1, prefCamNum = -1;
    size_t lowestIndex = prefCameras.size();
    for (const auto &cam : cameras) {
        // Iterate through prefCameras looking for matches
        for (size_t i = 0; i < lowestIndex; i++) {
            if (cam.second == prefCameras[i]) {
                deviceNum = cam.first;
                prefCamNum = static_cast<int>(i);
                lowestIndex = i;
                break;
            }
        }

        // If we matched the most preferred camera then we're done
        if (prefCamNum == 0) {
            break;
        }

        /*
        * Even if we didn't match any preferred camera, at least this is a
        * camera, so use as fallback
        */
        if (deviceNum == -1) {
            deviceNum = cam.first;
        }
    }

    if (deviceNum == -1) {
        throw std::runtime_error("No camera found");
    }

    LOG_INFO << "Selected camera #" << deviceNum << ": "
             << OS::Video::getCameraName(deviceNum);

    // SeeCam
    if(prefCamNum == 0) {
        auto see3cam = std::make_unique<See3CAM_CU40>("/dev/video" + std::to_string(deviceNum),
                                                      See3CAM_CU40::Resolution::_1280x720);
        // Run auto exposure algorithm
        const cv::Mat bubblescopeMask = See3CAM_CU40::createBubblescopeMask(see3cam->getSuperPixelSize());
        see3cam->autoExposure(bubblescopeMask);
        return see3cam;
    }
    // PixPro
    else if(prefCamNum == 1) {
        return std::make_unique<OpenCVInput>(deviceNum, cv::Size(1440, 1440), "pixpro_usb");
    }
    // Default
    else {
        return std::make_unique<OpenCVInput>(deviceNum, cv::Size(1280, 720), "webcam360");
    }
#endif
}
} // Video
} // BoBRobotics
