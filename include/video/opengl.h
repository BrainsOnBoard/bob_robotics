#pragma once

// BoB robotics includes
#include "input.h"

// OpenGL includes
#include <GL/glew.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace Video {
//----------------------------------------------------------------------------
// BoBRobotics::Video::OpenGL
//----------------------------------------------------------------------------
//! Read a video stream from an OpenGL buffer
class OpenGL : public Input
{
public:
    /*!
     * \brief Create a Video::Input for reading from an OpenGL window
     *
     * @param size Size of image
     * @param bottomLeft The starting coordinates to read from (from bottom left of screen)
     */
    OpenGL(const cv::Size &size, const cv::Point &bottomLeft = { 0, 0 })
      : m_Size(size)
      , m_BottomLeft(bottomLeft)
    {}

    //----------------------------------------------------------------------------
    // Input virtuals
    //----------------------------------------------------------------------------
    virtual std::string getCameraName() const override
    {
        return "opengl";
    }

    virtual cv::Size getOutputSize() const override
    {
        return m_Size;
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        // Make sure frame is of right size and type
        outFrame.create(m_Size, CV_8UC3);

        // Read pixels from framebuffer into outFrame
        // **TODO** it should be theoretically possible to go directly from frame buffer to GpuMat
        glReadPixels(m_BottomLeft.x, m_BottomLeft.y, m_Size.width, m_Size.height,
                     GL_BGR, GL_UNSIGNED_BYTE, outFrame.data);

        // Flip image vertically
        cv::flip(outFrame, outFrame, 0);

        return true;
    }

    virtual bool needsUnwrapping() const override
    {
        return false;
    }

private:
    const cv::Size m_Size;
    const cv::Point m_BottomLeft;
};
}   // namespace Video
}   // namespace BoBRobotics
