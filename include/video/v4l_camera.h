#pragma once

// Standard C++ includes
#include <functional>
#include <stdexcept>
#include <string>

// Standard C includes
#include <cstdint>

// Video4Linux includes
#include <linux/videodev2.h>

namespace BoBRobotics {
namespace Video {
//------------------------------------------------------------------------
// BoBRobotics::Video::Video4LinuxCamera
//------------------------------------------------------------------------
//! An interface for the low-level Video4Linux API
class Video4LinuxCamera
{
public:
    class Error
      : public std::runtime_error
    {
    public:
        Error(const std::string &msg);
    };

    Video4LinuxCamera();
    Video4LinuxCamera(const std::string &device,
                      unsigned int width,
                      unsigned int height,
                      uint32_t pixelFormat);
    ~Video4LinuxCamera();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void open(const std::string &device,
              unsigned int width,
              unsigned int height,
              uint32_t pixelFormat);
    void enumerateControls(std::function<void(const v4l2_queryctrl &)> processControl);
    void queryControl(uint32_t id, v4l2_queryctrl &queryControl);
    uint32_t capture(void *&buffer);
    int32_t getControlValue(uint32_t id) const;
    void setControlValue(uint32_t id, int32_t value);

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // File handle to camera
    int m_Camera;

    // Index of current frame - used to select correct buffer
    unsigned int m_Frame;

    // Two buffers and their corresponding information structures
    void *m_Buffer[2];
    v4l2_buffer m_BufferInfo[2];
}; // Video4LinuxCamera
} // Video
} // BoBRobotics
