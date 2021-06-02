#pragma once

// Opteran DevKit includes
#include "third_party/development_kit_odk2/devkit_driver/src/devkit_driver.h"

// BoB robotics includes
#include "common/macros.h"
#include "video/input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

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
    virtual ~ODK2();

    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual std::string getCameraName() const override;
    virtual cv::Size getOutputSize() const override;
    virtual bool readFrame(cv::Mat &outFrame) override;

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void readThread();

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    devkitDriverState_t m_State = {};

    devkitDriverOutputBufs_t m_OutputBufs = {};
    rawFrame_t m_RawFrameBuf = {};
    flowFrame_t m_FlowFrameBuf = {};
    rawCamera_t m_RawCameraBuf = {};
    imu_t m_IMUBuf;
    state_t m_StateBuf = {};

    uint64_t m_LastRawFrameTimestep = 0;

    std::thread m_ReadThread;
    std::mutex m_OutputBufMutex;
    std::atomic<bool> m_ShouldQuit;
}; // OpenCVInput
} // Video
} // BoBRobotics
