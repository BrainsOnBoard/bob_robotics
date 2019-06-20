#pragma once

// BoB robotics includes
#include "../common/logging.h"

// Standard C++ includes
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

// Standard C includes
#include <cstring>

// POSIX includes
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

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
        Error(const std::string &msg)
          : std::runtime_error(msg + " (" + strerror(errno) +")")
        {}
    };

    Video4LinuxCamera()
      : m_Camera(-1)
      , m_Frame(0)
      , m_Buffer{ nullptr, nullptr }
    {}

    Video4LinuxCamera(const std::string &device,
                      unsigned int width,
                      unsigned int height,
                      uint32_t pixelFormat)
      : Video4LinuxCamera()
    {
        open(device, width, height, pixelFormat);
    }

    ~Video4LinuxCamera()
    {
        if (m_Camera >= 0) {
            // Stop video streaming
            int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(m_Camera, VIDIOC_STREAMOFF, &type) < 0) {
                LOG_WARNING << "Could not stop streaming (" << strerror(errno)
                            << ")";
            }

            // munmap buffers
            if ((m_Buffer[0] && munmap(m_Buffer[0], m_BufferInfo[0].length) == -1) ||
                (m_Buffer[1] && munmap(m_Buffer[1], m_BufferInfo[1].length) == -1)) {
                LOG_WARNING << "Could not free buffers ("
                            << strerror(errno) << ")";
            }

            // Close camera
            close(m_Camera);
        }
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void open(const std::string &device,
              unsigned int width,
              unsigned int height,
              uint32_t pixelFormat)
    {
        // Open camera
        if ((m_Camera = ::open(device.c_str(), O_RDWR)) < 0) {
            throw Error("Could not open camera");
        }

        // Query capabilities
        v4l2_capability cap;
        if (ioctl(m_Camera, VIDIOC_QUERYCAP, &cap) < 0) {
            throw Error("Could not query capabilities");
        }

        LOG_INFO << "Opened device: " << cap.card;

        // Check required capabilities
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            throw Error("Device cannot capture video");
        }
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw Error("Device cannot stream");
        }

        // Fill format structure
        v4l2_format format;
        memset(&format, 0, sizeof(v4l2_format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.pixelformat = pixelFormat;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;

        // Set format
        if (ioctl(m_Camera, VIDIOC_S_FMT, &format) < 0) {
            throw Error("Cannot set format");
        }

        // Fill buffer request structure to request two buffers
        v4l2_requestbuffers bufferRequest;
        memset(&bufferRequest, 0, sizeof(v4l2_requestbuffers));
        bufferRequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferRequest.memory = V4L2_MEMORY_MMAP;
        bufferRequest.count = 2;

        if (ioctl(m_Camera, VIDIOC_REQBUFS, &bufferRequest) < 0) {
            throw Error("Cannot request buffers");
        }

        // Loop through buffers
        for (unsigned int i = 0; i < 2; i++) {
            // Fill buffer structure
            memset(&m_BufferInfo[i], 0, sizeof(v4l2_buffer));
            m_BufferInfo[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            m_BufferInfo[i].memory = V4L2_MEMORY_MMAP;
            m_BufferInfo[i].index = i;

            // Query buffers
            if (ioctl(m_Camera, VIDIOC_QUERYBUF, &m_BufferInfo[i]) < 0) {
                throw Error("Cannot query buffer");
            }

            LOG_DEBUG << "Queried " << m_BufferInfo[i].length << " byte buffer";

            // Map memory
            m_Buffer[i] = mmap(
                    nullptr,
                    m_BufferInfo[i]
                            .length, // Length of buffer returned from V4L
                    PROT_READ | PROT_WRITE, // Buffer is for RW access
                    MAP_SHARED, // Buffer is shared with other processes i.e.
                                // kernel driver
                    m_Camera,   // Camera device to map within
                    m_BufferInfo[i].m.offset); // Offset into device 'file'
                                               // where buffer should be mapped
            if (m_Buffer[i] == MAP_FAILED) {
                throw Error("Cannot mmap buffer");
            }

            // Zero buffer
            memset(m_Buffer[i], 0, m_BufferInfo[i].length);
        }

        // Start video streaming
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(m_Camera, VIDIOC_STREAMON, &type) < 0) {
            throw Error("Cannot start streaming");
        }

        // Enqueue our buffer onto the device's incoming queue
        if (ioctl(m_Camera, VIDIOC_QBUF, &m_BufferInfo[0]) < 0) {
            throw Error("Cannot enqueue buffer");
        }

        // Ensure that frame is zeroed
        m_Frame = 0;
    }

    void enumerateControls(std::function<void(const v4l2_queryctrl &)> processControl)
    {
        // Fill control query structure
        v4l2_queryctrl queryControl;
        memset(&queryControl, 0, sizeof(v4l2_queryctrl));
        queryControl.id = V4L2_CTRL_FLAG_NEXT_CTRL;

        while (ioctl(m_Camera, VIDIOC_QUERYCTRL, &queryControl) == 0) {
            // If this control isn't disabled
            if (!(queryControl.flags & V4L2_CTRL_FLAG_DISABLED)) {
                processControl(queryControl);
            }

            queryControl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
        }
        if (errno != EINVAL) {
            throw Error("Cannot query controls");
        }
    }

    void queryControl(uint32_t id, v4l2_queryctrl &queryControl)
    {
        // Fill control query structure
        memset(&queryControl, 0, sizeof(v4l2_queryctrl));
        queryControl.id = id;

        if (ioctl(m_Camera, VIDIOC_QUERYCTRL, &queryControl) < 0) {
            throw Error("Cannot query controls");
        }
    }

    uint32_t capture(void *&buffer)
    {
        // Dequeue this frame's buffer from device's outgoing queue
        if (ioctl(m_Camera, VIDIOC_DQBUF, &m_BufferInfo[m_Frame]) < 0) {
            throw Error("Cannot dequeue buffer");
        }

        // Pass out buffer data and length
        buffer = m_Buffer[m_Frame];
        uint32_t sizebytes = m_BufferInfo[m_Frame].length;

        // Increment frame number
        m_Frame = (m_Frame + 1) % 2;

        // Enqueue next buffer onto the device's incoming queue
        if (ioctl(m_Camera, VIDIOC_QBUF, &m_BufferInfo[m_Frame]) < 0) {
            throw Error("Cannot enqueue buffer");
        }

        return sizebytes;
    }

    int32_t getControlValue(uint32_t id) const
    {
        // Fill control structure
        v4l2_control control;
        memset(&control, 0, sizeof(v4l2_control));
        control.id = id;

        // Get control value
        if (ioctl(m_Camera, VIDIOC_G_CTRL, &control) < 0) {
            throw Error("Cannot get control value");
        } else {
            return control.value;
        }
    }

    void setControlValue(uint32_t id, int32_t value)
    {
        // Fill control structure
        v4l2_control control;
        memset(&control, 0, sizeof(v4l2_control));
        control.id = id;
        control.value = value;

        // Get control value
        if (ioctl(m_Camera, VIDIOC_S_CTRL, &control) < 0) {
            throw Error("Cannot set control value");
        }
    }

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
