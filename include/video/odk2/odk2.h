#pragma once

// Opteran DevKit includes
extern "C" {
#include "devkit_driver/src/devkit_driver.h"
}

// BoB robotics includes
#include "common/macros.h"
#include "video/input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Video {

//----------------------------------------------------------------------------
// BoBRobotics::Video::ODK2
//----------------------------------------------------------------------------
//! A thin wrapper for reading cylindrically unwrapped data from Opteran DevKit
class ODK2 : public Input
{
public:
    //! Create a new video stream, using the default given by OpenCV
    ODK2(const std::string &hostIP = "192.168.2.1", const std::string &remoteIP = "192.168.2.9");

    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual std::string getCameraName() const override;
    virtual cv::Size getOutputSize() const override;
    virtual bool readFrame(cv::Mat &outFrame) override;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    devkitDriverState_t m_State = {};
    devkitDriverOutputBufs_t m_OutputBufs = {};
    rawFrame_t m_RawFrameBuf = {};

    cv::Mat m_RawFrameUnwrapped;
    /*flowFrame_t flowFrameBuf = {0};
    state_t stateBuf = {0};*/
}; // OpenCVInput
} // Video
} // BoBRobotics
