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
#include "../third_party/path.h"

namespace GeNNRobotics {
namespace Video {
class Input
{
public:
    virtual ~Input()
    {}

    virtual void setOutputSize(const cv::Size &outSize)
    {}

    virtual ImgProc::OpenCVUnwrap360 createDefaultUnwrapper(const cv::Size &unwrapRes)
    {
        // Create unwrapper
        ImgProc::OpenCVUnwrap360 unwrapper(getOutputSize(), unwrapRes);

        const std::string name = getCameraName();
        const std::string fileName = name + ".yaml";
        filesystem::path filePath(fileName);

        // first check if file exists in working directory
        if (!filePath.exists()) {
            // next check if there is a local GeNN_Robotics folder (i.e. git
            // submodule)
            const filesystem::path paramsDir = filesystem::path("imgproc") / "unwrapparams";

            filePath = filesystem::path("GeNN_Robotics") / paramsDir / fileName;
            if (!filePath.exists()) {
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

                filePath = filesystem::path(env) / paramsDir / fileName;
                if (!filePath.exists()) {
                    throw std::runtime_error(
                            "Could not find unwrap parameters file for " +
                            name);
                }
            }
        }

        // read unwrap parameters from file
        std::cout << "Loading unwrap parameters from " << filePath.str() << std::endl;
        cv::FileStorage fs(filePath.str(), cv::FileStorage::READ);
        unwrapper << fs;
        fs.release();
        return unwrapper;
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
} // GeNNRobotics
