#pragma once

// BoB robotics includes
#include "video/input.h"

// Standard C++ includes
#include <string>

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
    OpenGL(const cv::Size &size, const cv::Point &bottomLeft = { 0, 0 });

    //----------------------------------------------------------------------------
    // Input virtuals
    //----------------------------------------------------------------------------
    virtual std::string getCameraName() const override;
    virtual cv::Size getOutputSize() const override;
    virtual bool readFrame(cv::Mat &outFrame) override;
    virtual bool needsUnwrapping() const override;

private:
    const cv::Size m_Size;
    const cv::Point m_BottomLeft;
};
}   // namespace Video
}   // namespace BoBRobotics
