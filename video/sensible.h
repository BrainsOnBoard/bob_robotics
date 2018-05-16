#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "opencvinput.h"
#include "input.h"
#ifndef _WIN32
#include "common/see3cam_cu40.h"
#include "os/video.h"
#endif

namespace Video {
Input *
getSensibleCamera()
{
#ifdef _WIN32
    // for Windows we currently just select the first camera
    int deviceNum = 0;
    int prefCamNum = -1;
#else
    // get vector of video input devices on system
    auto cameras = OS::Video::getCameras();

    // list of preferred cameras in priority order
    static const std::array<std::string, 3> prefCameras{ SEE3CAM_DEVICE_NAME,
                                                         PIXPRO_USB_DEVICE_NAME,
                                                         WEBCAM360_DEVICE_NAME };

    int deviceNum = -1, prefCamNum = -1;
    size_t lowestIndex = prefCameras.size();
    for (OS::Video::CameraDevice cam : cameras) {
        // Iterate through prefCameras looking for matches
        for (size_t i = 0; i < lowestIndex; i++) {
            if (cam.name == prefCameras[i]) {
                deviceNum = cam.number;
                prefCamNum = i;
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
            deviceNum = cam.number;
        }
    }
#endif

    // std::cout << "Selected: " << deviceNum << ": " << prefCamNum << std::endl;

    if (deviceNum == -1) {
        throw std::runtime_error("No camera found");
    }
    switch (prefCamNum) {
    case 0: // SeeCam
        return new See3CAM_CU40("/dev/video" + std::to_string(deviceNum),
                            See3CAM_CU40::Resolution::_1280x720,
                            cv::Size(640, 360),
                            20);
    case 1: // PixPro
        return new OpenCVInput(deviceNum, cv::Size(1440, 1440), "pixpro_usb");
    default: // webcam with panoramic lens
        return new OpenCVInput(deviceNum, cv::Size(1280, 720), "webcam360");
    }
}
}
