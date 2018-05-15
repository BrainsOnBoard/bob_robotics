#pragma once

// C++ includes
#include <iostream>
#include <stdexcept>

// opencv
#include <opencv2/opencv.hpp>

// common includes
#include "../common/opencv_unwrap_360.h"
#include "../os/filesystem.h"

namespace VideoIn {
class VideoInput
{
public:
    virtual ~VideoInput()
    {}

    virtual void setOutputSize(const cv::Size &outSize)
    {}

    virtual const std::string getDefaultUnwrapParams()
    {
        return "";
    }

    virtual void createDefaultUnwrapper(OpenCVUnwrap360 &unwrapper)
    {
        // tell the unwrapper this camera's resolution
        unwrapper.m_CameraResolution = getOutputSize();

        // first try to read params from file with default name
        const std::string name = getCameraName();
        const std::string fileName = name + ".yaml";
        if (OS::FileSystem::fileExists(fileName)) {
            // read params from file into unwrapper
            std::cout << "Loading unwrap parameters from " << fileName
                      << std::endl;
            cv::FileStorage fs(fileName, cv::FileStorage::READ);
            unwrapper << fs;
            fs.release();
        } else {
            // if the file doesn't exist, fall back on defaults
            std::string paramString = getDefaultUnwrapParams();
            if (paramString == "") {
                throw std::runtime_error("Camera " + name +
                                         " does not have an unwrap params file "
                                         "or default values");
            }

            // read params from paramString into unwrapper
            cv::FileStorage fs(".yml",
                               cv::FileStorage::READ | cv::FileStorage::MEMORY);
            unwrapper << fs;
            fs.release();
        }
    }

    virtual bool readFrame(cv::Mat &outFrame) = 0;
    virtual const std::string getCameraName() = 0;
    virtual cv::Size getOutputSize() const = 0;
};
}
