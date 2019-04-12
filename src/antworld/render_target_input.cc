// BoB robotics includes
#include "antworld/render_target.h"
#include "antworld/render_target_input.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetInput
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
cv::Size RenderTargetInput::getOutputSize() const
{
    return cv::Size(m_RenderTarget.getWidth(), m_RenderTarget.getHeight());
}
//----------------------------------------------------------------------------
bool RenderTargetInput::readFrame(cv::Mat &outFrame)
{
    // Make sure frame is of right size and type
    outFrame.create(m_RenderTarget.getHeight(), m_RenderTarget.getWidth(), CV_8UC3);

    // Bind render target texture
    glBindTexture(GL_TEXTURE_2D, m_RenderTarget.getTexture());

    // Read texture into outframe
    glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, outFrame.data);

    // Flip image vertically
    cv::flip(outFrame, outFrame, 0);

    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);

    return true;
}
}   // namespace AntWorld
}   // namespace BoBRobotics
