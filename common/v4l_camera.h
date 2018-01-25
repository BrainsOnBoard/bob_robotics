#pragma once

// Standard C++ includes
#include <functional>
#include <iostream>

// Standard C includes
#include <cstring>

// Posix includes
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

// Video4Linux includes
#include <linux/videodev2.h>

//------------------------------------------------------------------------
// Video4LinuxCamera
//------------------------------------------------------------------------
class Video4LinuxCamera
{
public:
    Video4LinuxCamera() : m_Camera(-1), m_Frame(0), m_Buffer{nullptr, nullptr}
    {
    }

    Video4LinuxCamera(const std::string &device, unsigned int width, unsigned int height, uint32_t pixelFormat) : Video4LinuxCamera()
    {
        if(!open(device, width, height, pixelFormat)) {
            throw std::runtime_error("Cannot open camera");
        }
    }

    ~Video4LinuxCamera()
    {
        if(m_Camera >= 0) {
            // Stop video streaming
            int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if(ioctl(m_Camera, VIDIOC_STREAMOFF, &type) < 0) {
                std::cerr << "ERROR: Cannot stop streaming (" << strerror(errno) << ")" << std::endl;
            }

            // munmap buffers
            if(munmap(m_Buffer[0], m_BufferInfo[0].length) == -1 || munmap(m_Buffer[1], m_BufferInfo[1].length) == -1) {
                std::cerr << "ERROR: Could not free buffers (" << strerror(errno) << ")" << std::endl;
            }

            // Close camera
            close(m_Camera);
        }
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool open(const std::string &device, unsigned int width, unsigned int height, uint32_t pixelFormat)
    {
        // Open camera
        if((m_Camera = ::open(device.c_str(), O_RDWR)) < 0) {
            return false;
        }

        // Query capabilities
        v4l2_capability cap;
        if(ioctl(m_Camera, VIDIOC_QUERYCAP, &cap) < 0) {
            std::cerr << "ERROR: Cannot query capabilities (" << strerror(errno) << ")" << std::endl;
            return false;
        }

        std::cout << "Opened device: " << cap.card << std::endl;

        // Check required capabilities
        if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            std::cerr << "ERROR: Device cannot capture video (" << strerror(errno) << ")" << std::endl;
            return false;
        }
        if(!(cap.capabilities & V4L2_CAP_STREAMING)) {
            std::cerr << "ERROR: Device cannot stream (" << strerror(errno) << ")" << std::endl;
            return false;
        }

        // Fill format structure
        v4l2_format format;
        memset(&format, 0, sizeof(v4l2_format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.pixelformat = pixelFormat;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;

        // Set format
        if(ioctl(m_Camera, VIDIOC_S_FMT, &format) < 0) {
            std::cerr << "ERROR: Cannot set format (" << strerror(errno) << ")" << std::endl;
            return false;
        }

        // Fill buffer request structure to request two buffers
        v4l2_requestbuffers bufferRequest;
        memset(&bufferRequest, 0, sizeof(v4l2_requestbuffers));
        bufferRequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferRequest.memory = V4L2_MEMORY_MMAP;
        bufferRequest.count = 2;

        if(ioctl(m_Camera, VIDIOC_REQBUFS, &bufferRequest) < 0){
            std::cerr << "ERROR: Cannot request buffers (" << strerror(errno) << ")" << std::endl;
            return false;
        }

        // Loop through buffers
        for(unsigned int i = 0; i < 2; i++) {
            // Fill buffer structure
            memset(&m_BufferInfo[i], 0, sizeof(v4l2_buffer));
            m_BufferInfo[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            m_BufferInfo[i].memory = V4L2_MEMORY_MMAP;
            m_BufferInfo[i].index = i;

            // Query buffers
            if(ioctl(m_Camera, VIDIOC_QUERYBUF, &m_BufferInfo[i]) < 0) {
                std::cerr << "ERROR: Cannot query buffer (" << strerror(errno) << ")" << std::endl;
                return false;
            }

            std::cout << "Queried " << m_BufferInfo[i].length << " byte buffer" << std::endl;

            // Map memory
            m_Buffer[i] = mmap(nullptr,
                               m_BufferInfo[i].length,      // Length of buffer returned from V4L
                               PROT_READ | PROT_WRITE,      // Buffer is for RW access
                               MAP_SHARED,                  // Buffer is shared with other processes i.e. kernel driver
                               m_Camera,                    // Camera device to map within
                               m_BufferInfo[i].m.offset);   // Offset into device 'file' where buffer should be mapped
            if(m_Buffer[i] == MAP_FAILED) {
                std::cerr << "ERROR: Cannot mmap buffer (" << strerror(errno) << ")" << std::endl;
                return false;
            }

            // Zero buffer
            memset(m_Buffer[i], 0, m_BufferInfo[i].length);
        }

        // Start video streaming
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if(ioctl(m_Camera, VIDIOC_STREAMON, &type) < 0) {
            std::cerr << "ERROR: Cannot start streaming (" << strerror(errno) << ")" << std::endl;
            return false;
        }

        // Enqueue our buffer onto the device's incoming queue
        if(ioctl(m_Camera, VIDIOC_QBUF, &m_BufferInfo[0]) < 0) {
            std::cerr << "ERROR: Cannot enqueue buffer (" << strerror(errno) << ")" << std::endl;
            return false;
        }

        // Ensure that frame is zeroed
        m_Frame = 0;

        return true;
    }

    bool enumerateControls(std::function<void(const v4l2_queryctrl&)> processControl)
    {
        // Fill control query structure
        v4l2_queryctrl queryControl;
        memset(&queryControl, 0, sizeof(v4l2_queryctrl));
        queryControl.id = V4L2_CTRL_FLAG_NEXT_CTRL;

        while(ioctl(m_Camera, VIDIOC_QUERYCTRL, &queryControl) == 0) {
            // If this control isn't disabled
            if (!(queryControl.flags & V4L2_CTRL_FLAG_DISABLED)) {
                processControl(queryControl);
            }

            queryControl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
        }
        if(errno == EINVAL) {
            return true;
        }
        else {
            std::cerr << "ERROR: Cannot query controls (" << strerror(errno) << ")" << std::endl;
            return false;
        }
    }

    bool queryControl(uint32_t id, v4l2_queryctrl &queryControl)
    {
        // Fill control query structure
        memset(&queryControl, 0, sizeof(v4l2_queryctrl));
        queryControl.id = id;

        if(ioctl(m_Camera, VIDIOC_QUERYCTRL, &queryControl) == 0) {
            return true;
        }
        else {
            std::cerr << "ERROR: Cannot query controls (" << strerror(errno) << ")" << std::endl;
            return false;
        }

    }

    bool capture(void *&buffer, uint32_t &sizeBytes)
    {
        // Dequeue this frame's buffer from device's outgoing queue
        if(ioctl(m_Camera, VIDIOC_DQBUF, &m_BufferInfo[m_Frame]) < 0) {
            std::cerr << "ERROR: Cannot dequeue buffer (" << strerror(errno) << ")" << std::endl;
            return false;
        }

        // Pass out buffer data and length
        buffer = m_Buffer[m_Frame];
        sizeBytes = m_BufferInfo[m_Frame].length;

        // Increment frame number
        m_Frame = (m_Frame + 1) % 2;

        // Enqueue next buffer onto the device's incoming queue
        if(ioctl(m_Camera, VIDIOC_QBUF, &m_BufferInfo[m_Frame]) < 0) {
            std::cerr << "ERROR: Cannot enqueue buffer (" << strerror(errno) << ")" << std::endl;
            return false;
        }



        return true;
    }

    bool getControlValue(uint32_t id, int32_t &value)
    {
        // Fill control structure
        v4l2_control control;
        memset(&control, 0, sizeof(v4l2_control));
        control.id = id;

        // Get control value
        if(ioctl(m_Camera, VIDIOC_G_CTRL, &control) < 0) {
            std::cerr << "ERROR: Cannot get control value (" << strerror(errno) << ")" << std::endl;
            return false;
        }
        else {
            value = control.value;
            return true;
        }

    }

    bool setControlValue(uint32_t id, int32_t value)
    {
        // Fill control structure
        v4l2_control control;
        memset(&control, 0, sizeof(v4l2_control));
        control.id = id;
        control.value = value;

        // Get control value
        if(ioctl(m_Camera, VIDIOC_S_CTRL, &control) < 0) {
            std::cerr << "ERROR: Cannot set control value (" << strerror(errno) << ")" << std::endl;
            return false;
        }
        else {
            return true;
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
};