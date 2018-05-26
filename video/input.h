#pragma once

// C++ includes
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

// opencv
#include <opencv2/opencv.hpp>

// GeNNRobotics includes
#include "../imgproc/opencv_unwrap_360.h"
#include "../os/filesystem.h"

namespace GeNNRobotics {
namespace Video {
#define DefaultCameraName "unknown_camera"

class Input
{
public:
    virtual ~Input()
    {}

    virtual ImgProc::OpenCVUnwrap360 createDefaultUnwrapper(const cv::Size &unwrapRes)
    {
        // Create unwrapper
        ImgProc::OpenCVUnwrap360 unwrapper(getOutputSize(), unwrapRes);

        const std::string name = getCameraName();
        const std::string fileName = name + ".yaml";
        std::string filePath = fileName;

        // first check if file exists in working directory
        if (!OS::FileSystem::fileExists(filePath)) {
            // next check if there is a local GeNN_Robotics folder (i.e. git
            // submodule)
            static const std::string paramsDir = "/imgproc/unwrapparams/";
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
        return unwrapper;
    }

    virtual const std::string getCameraName() const
    {
        return DefaultCameraName;
    }

    virtual cv::Size getOutputSize() const
    {
        return cv::Size();
    }

    virtual bool needsUnwrapping() const
    {
        return false;
    }

    virtual bool readFrame(cv::Mat &outFrame) = 0;

    virtual void setOutputSize(const cv::Size &outSize)
    {}

protected:
    /*
     * Default constructor is protected to stop users directly creating objects
     * of this class.
     */
    Input()
    {}
}; // Input
} // Video
} // GeNNRobotics
