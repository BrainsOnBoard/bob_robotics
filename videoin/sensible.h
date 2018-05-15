#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "opencvinput.h"
#include "videoinput.h"
#ifndef _WIN32
#include "common/see3cam_cu40.h"
#include "os/video.h"
#endif

namespace VideoIn {
VideoInput *
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
    static const std::array<std::string, 3> prefCameras{ "See3CAM_CU40",
                                                         "PIXPRO SP360 4K",
                                                         "USB 2.0 Camera" };

    int deviceNum = -1, prefCamNum = -1;
    size_t lowestIndex = prefCameras.size();
    for (OS::Video::CameraDevice cam : cameras) {
        /*
         * Strip anything after colon from name. Newer kernels seem to add
         * a colon plus the name repeated for some reason.
         */
        size_t colon = cam.name.find(':');
        if (colon != std::string::npos) {
            cam.name = cam.name.substr(0, colon);
        }

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
