#pragma once

// Opteran DevKit includes
#include "third_party/development_kit_odk2/devkit_driver/src/devkit_driver.h"

// Third-party includes
#include "third_party/units.h"

// BoB robotics includes
#include "common/macros.h"
#include "video/input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <array>
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
    using degree_t = units::angle::degree_t;
    using radian_t = units::angle::radian_t;
    using turn_t = units::angle::turn_t;

public:
    //! Create a new video stream, using the default given by OpenCV
    ODK2(const std::string &hostIP = "192.168.2.1", const std::string &remoteIP = "192.168.2.9");
    virtual ~ODK2();

    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual std::string getCameraName() const override{ return "Opteran DevKit"; }
    virtual cv::Size getOutputSize() const override{ return cv::Size(BAND_WIDTH, BAND_HEIGHT); }
    virtual bool readFrame(cv::Mat &outFrame) override;
    virtual bool needsUnwrapping() const override{ return false; }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    //! Get Euler angles from ODK2 IMU
    std::array<degree_t, 3> getEulerAngles() const;

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    // Thread function for reading streaming data off ODK2
    void readThread();

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // ODK2 state and buffers
    // **NOTE** m_OutputBufs and all individual buffers should be accessed via m_OutputBufsMutex
    devkitDriverState_t m_State = {};
    devkitDriverOutputBufs_t m_OutputBufs = {};

    // Individual ODK2 buffers
    rawFrame_t m_RawFrameBuf = {};
    flowFrame_t m_FlowFrameBuf = {};
    rawCamera_t m_RawCameraBuf = {};
    imu_t m_IMUBuf;
    state_t m_StateBuf = {};

    // Last raw frame timestep
    uint64_t m_LastRawFrameTimestep = 0;

    // Read thread
    std::thread m_ReadThread;

    // Mutex used to guard output buffers
    mutable std::mutex m_OutputBufsMutex;

    // Atomic flag used to signal read thread to quit
    std::atomic<bool> m_ShouldQuit;
}; // OpenCVInput
} // Video
} // BoBRobotics
