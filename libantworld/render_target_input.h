#pragma once

// BoB robotics includes
#include "video/input.h"

// OpenGL includes
#include <GL/glew.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetInput
//----------------------------------------------------------------------------
//! Read a video stream from a libantworld RenderTarget
namespace BoBRobotics
{
namespace AntWorld
{
class RenderTarget;

class RenderTargetInput : public Video::Input
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
    RenderTargetInput(RenderTarget &renderTarget, bool needsUnwrapping = false)
        : m_RenderTarget(renderTarget), m_NeedsUnwrapping(needsUnwrapping)
    {}

    //----------------------------------------------------------------------------
    // Input virtuals
    //----------------------------------------------------------------------------
    virtual std::string getCameraName() const override
    {
        return "render_target";
    }

    virtual cv::Size getOutputSize() const override;

    virtual bool readFrame(cv::Mat &outFrame) override;

    virtual bool needsUnwrapping() const override
    {
        return m_NeedsUnwrapping;
    }

private:
    //----------------------------------------------------------------------------
    // Protected members
    //----------------------------------------------------------------------------
    RenderTarget &m_RenderTarget;
    const bool m_NeedsUnwrapping;
};
}   // namespace AntWorld
}   // namespace BoBRobotics