#pragma once

// Standard C++ includes
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoBRobotics includes
#include "../imgproc/opencv_unwrap_360.h"
#include "../third_party/path.h"

namespace BoBRobotics {
namespace Video {
#define DefaultCameraName "unknown_camera"

//----------------------------------------------------------------------------
// BoBRobotics::Video::Input
//----------------------------------------------------------------------------
//! An abstract class representing a video input stream
class Input
{
public:
    virtual ~Input()
    {}

    /*!
     * \brief Create an ImgProc::OpenCVUnwrap360 object for this video stream
     * 
     * @param args The resolution of the unwrapped image, as cv::Size or two ints
     */
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

    /*!
     * \brief Get the name of this type of camera as a (short) string
     * 
     * Note that this is used to load the appropriate unwrapping parameters
     * (we look for a file called [camera name].yaml).
     */
    virtual const std::string getCameraName() const
    {
        return DefaultCameraName;
    }

    /*!
     * \brief Try to read a frame in greyscale from this video source
     * 
     * @return Whether a new frame was read
     */
    virtual bool readGreyscaleFrame(cv::Mat &outFrame)
    {
        // If reading (colour frame) was succesful
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

    //! Whether this video source needs unwrapping with an ImgProc::OpenCVUnwrap360
    virtual bool needsUnwrapping() const
    {
        // only panoramic cameras are defined with the camera name specified
        return getCameraName() != DefaultCameraName;
    }

    //! Set the output resolution of this video stream
    virtual void setOutputSize(const cv::Size &)
    {
        throw std::runtime_error("This camera's resolution cannot be changed at runtime");
    }

    //! Get the current output resolution of this video stream
    virtual cv::Size getOutputSize() const = 0;

    /*!
     * \brief Try to read a frame in colour from this video source
     * 
     * @return Whether a new frame was read
     */
    virtual bool readFrame(cv::Mat &outFrame) = 0;

private:
    cv::Mat m_IntermediateFrame;
}; // Input
} // Video
} // BoBRobotics
