// BoB robotics includes
#include "video/opengl/opengl.h"

// OpenGL includes
#include <GL/glew.h>

namespace BoBRobotics {
namespace Video {

OpenGL::OpenGL(const cv::Size &size, const cv::Point &bottomLeft)
    : m_Size(size)
    , m_BottomLeft(bottomLeft)
{}

std::string OpenGL::getCameraName() const
{
    return "opengl";
}

cv::Size OpenGL::getOutputSize() const
{
    return m_Size;
}

bool OpenGL::readFrame(cv::Mat &outFrame)
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

bool OpenGL::needsUnwrapping() const
{
    return false;
}

}   // namespace Video
}   // namespace BoBRobotics
