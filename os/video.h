#pragma once

// C++ includes
#include <iostream>
#include <string>
#include <vector>

// Linux includes
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace BoBRobotics {
namespace OS {
namespace Video {
typedef std::pair<int, std::string> CameraDevice;

inline const std::string getCameraName(int deviceNumber)
{
    int fd = open(("/dev/video" + std::to_string(deviceNumber)).c_str(), O_RDONLY);
    if (fd == -1) {
        return "";
    }

    v4l2_capability video_cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &video_cap) == -1) {
        std::cerr << "Warning: Could not get video device capabilities"
                  << std::endl;
        close(fd);
        return "";
    }

    // Close file descriptor
    close(fd);

    /*
     * Strip colon and everything after from name. Newer kernels seem to add
     * a colon plus the name repeated for some reason.
     */
    for (__u8 *c = video_cap.card; c; c++) {
        if (*c == ':') {
            *c = '\0';
            break;
        }
    }

    return std::string((char *) video_cap.card);
}

inline std::vector<CameraDevice> getCameras()
{
    std::vector<CameraDevice> cameras;
    for (int i = 0; i < 64; i++) {
        const std::string name = getCameraName(i);
        if (!name.empty()) {
            cameras.emplace_back(i, name);
        }
    }
    return cameras;
}
} // Video
} // OS
} // BoBRobotics
