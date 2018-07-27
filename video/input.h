#pragma once

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoBRobotics includes
#include "../imgproc/opencv_unwrap_360.h"
#include "../third_party/path.h"

namespace BoBRobotics {
namespace Video {
#define DefaultCameraName "unknown_camera"

class Input
{
public:
    virtual ~Input()
    {}

    template<typename... Ts>
    ImgProc::OpenCVUnwrap360 createUnwrapper(Ts &&... args)
    {
        cv::Size unwrapRes(std::forward<Ts>(args)...);

        // Create unwrapper
        ImgProc::OpenCVUnwrap360 unwrapper(getOutputSize(), unwrapRes);

        const std::string name = getCameraName();
        const std::string fileName = name + ".yaml";
        filesystem::path filePath(fileName);

        // first check if file exists in working directory
        if (!filePath.exists()) {
            // next check if there is a local bob_robotics folder (i.e. git
            // submodule)
            const filesystem::path paramsDir = filesystem::path("imgproc") / "unwrapparams";

            filePath = filesystem::path("bob_robotics") / paramsDir / fileName;
            if (!filePath.exists()) {
                // lastly look for environment variable pointing to
                // bob_robotics
                static const char *envVarName = "BOB_ROBOTICS_PATH";
                const char *env = std::getenv(envVarName);
                if (!env) {
                    throw std::runtime_error(std::string(envVarName) +
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
        fs["unwrapper"] >> unwrapper;
        fs.release();
        return unwrapper;
    }

    virtual const std::string getCameraName() const
    {
        return DefaultCameraName;
    }

    virtual cv::Size getOutputSize() const = 0;
    virtual bool readFrame(cv::Mat &outFrame) = 0;

    virtual bool readGreyscaleFrame(cv::Mat &outFrame)
    {
        // If reading (RGB frame) was succesful
        if(readFrame(m_IntermediateFrame)) {
            // If output frame isn't correct size, create it
            if(outFrame.size() != m_IntermediateFrame.size()) {
                outFrame.create(m_IntermediateFrame.size(), CV_8UC1);
            }

            // Convert intermediate frame to greyscale
            cv::cvtColor(m_IntermediateFrame, outFrame, CV_BGR2GRAY);
            return true;
        }
        else {
            return false;
        }
    }

    virtual void readFrameSync(cv::Mat &outFrame)
    {
        while (!readFrame(outFrame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    virtual bool needsUnwrapping() const
    {
        // only panoramic cameras are defined with the camera name specified
        return getCameraName() != DefaultCameraName;
    }

    virtual void setOutputSize(const cv::Size &)
    {
        throw std::runtime_error("This camera's resolution cannot be changed at runtime");
    }

private:
    cv::Mat m_IntermediateFrame;
}; // Input
} // Video
} // BoBRobotics
