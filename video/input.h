#pragma once

// C++ includes
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

// opencv
#include <opencv2/opencv.hpp>

// common includes
#include "../common/opencv_unwrap_360.h"
#include "../os/filesystem.h"

namespace GeNN_Robotics {
namespace Video {
class Input
{
public:
    virtual ~Input()
    {}

    virtual void setOutputSize(const cv::Size &outSize)
    {}

    virtual void createDefaultUnwrapper(OpenCVUnwrap360 &unwrapper)
    {
        // tell the unwrapper this camera's resolution
        unwrapper.m_CameraResolution = getOutputSize();

        const std::string name = getCameraName();
        const std::string fileName = name + ".yaml";
        std::string filePath = fileName;

        // first check if file exists in working directory
        if (!OS::FileSystem::fileExists(filePath)) {
            // next check if there is a local GeNN_Robotics folder (i.e. git
            // submodule)
            static const std::string paramsDir = "/unwrap/defaultparams/";
            filePath = "GeNN_Robotics" + paramsDir + fileName;
            if (!OS::FileSystem::fileExists(filePath)) {
                // lastly look for environment variable pointing to
                // GeNN_Robotics
                static const char *envVarName = "GENN_ROBOTICS_PATH";
                const char *env = std::getenv(envVarName);
                if (!env) {
                    throw std::runtime_error(
                            std::string(envVarName) +
                            " environment variable is not set and unwrap "
                            "parameters file could not be found locally");
                }

                filePath = env + paramsDir + fileName;
                if (!OS::FileSystem::fileExists(filePath)) {
                    throw std::runtime_error(
                            "Could not find unwrap parameters file for " +
                            name);
                }
            }
        }

        // read unwrap parameters from file
        std::cout << "Loading unwrap parameters from " << filePath << std::endl;
        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        unwrapper << fs;
        fs.release();
    }

    virtual const std::string getCameraName() const
    {
        return "unknown_camera";
    }

    virtual bool readFrame(cv::Mat &outFrame)
    {
        return false;
    }

    virtual cv::Size getOutputSize() const
    {
        return cv::Size();
    }

protected:
    /*
     * Default constructor is protected to stop users directly creating objects
     * of this class.
     */
    Input()
    {}
}; // Input
} // Video
} // GeNN_Robotics