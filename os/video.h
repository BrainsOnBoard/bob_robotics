#pragma once

#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <vector>

namespace OS::Video {
struct CameraDevice
{
    int number;
    std::string name;
};

const std::string
getCameraName(int deviceNumber)
{
    int fd;
    if ((fd = open(("/dev/video" + std::to_string(deviceNumber)).c_str(),
                   O_RDONLY)) == -1) {
        return "";
    }

    struct v4l2_capability video_cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &video_cap) == -1) {
        std::cerr << "Warning: Could not get video device capabilities"
                  << std::endl;
        close(fd);
        return "";
    }

    return std::string((char *) video_cap.card);
}

std::vector<CameraDevice>
getCameras()
{
    std::vector<CameraDevice> cameras;
    int fd;
    struct v4l2_capability video_cap;
    for (int i = 0; i < 64; i++) {
        const std::string name = getCameraName(i);
        if (name == "") {
            continue;
        }

        CameraDevice dev;
        dev.number = i;
        dev.name = name;
        cameras.push_back(dev);
        close(fd);
    }
    return cameras;
}
}
