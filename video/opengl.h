#pragma once

// OpenGL includes
#include <GL/glew.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "input.h"

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
     * @param readX, readY The starting coordinates to read from (from bottom left of screen)
     * @param readWidth, readHeight Size of image
     *
     * **NOTE** intentionally NOT using a cv::Rect as OpenGL coordinates are (typically)
     * from bottom left of screen which would require window size etc. to convert
     */
    OpenGL(GLint readX,  GLint readY,  GLsizei readWidth,  GLsizei readHeight)
      : m_ReadX(readX), m_ReadY(readY), m_ReadWidth(readWidth), m_ReadHeight(readHeight)
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
        return cv::Size(m_ReadWidth, m_ReadHeight);
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        // Make sure frame is of right size and type
        outFrame.create(m_ReadWidth, m_ReadHeight, CV_8UC3);

        // Read pixels from framebuffer into outFrame
        // **TODO** it should be theoretically possible to go directly from frame buffer to GpuMat
        glReadPixels(m_ReadX, m_ReadY, m_ReadWidth, m_ReadHeight,
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
    //----------------------------------------------------------------------------
    // Protected members
    //----------------------------------------------------------------------------
    const GLint m_ReadX;
    const GLint m_ReadY;
    const GLsizei m_ReadWidth;
    const GLsizei m_ReadHeight;
};
}   // namespace Video
}   // namespace BoBRobotics