// BoB robotics includes
#include "antworld/render_target.h"

// Standard C++ includes
#include <stdexcept>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTarget
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RenderTarget::RenderTarget(GLsizei width, GLsizei height)
:   m_Width(width), m_Height(height)
{
    // Create FBO for rendering to cubemap and bind
    glGenFramebuffers(1, &m_FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);

    // Create texture and bind
    // **TODO** it would be better to use native format
    glGenTextures(1, &m_Texture);
    glBindTexture(GL_TEXTURE_2D, m_Texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // Attach frame texture to frame buffer
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, m_Texture, 0);

    // Set draw buffers
    const GLenum drawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, drawBuffers);

    // Create depth render buffer
    glGenRenderbuffers(1, &m_DepthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_DepthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);

    // Attach depth buffer to frame buffer
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_DepthBuffer);

    // Check frame buffer is created correctly
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("Frame buffer not complete");
    }

    // Unbind cube map and frame buffer
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
//----------------------------------------------------------------------------
RenderTarget::~RenderTarget()
{
    glDeleteRenderbuffers(1, &m_DepthBuffer);
    glDeleteTextures(1, &m_Texture);
    glDeleteFramebuffers(1, &m_FBO);
}
//----------------------------------------------------------------------------
void RenderTarget::bind()
{
    // Bind the cubemap FBO for offscreen rendering
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);
}
//----------------------------------------------------------------------------
void RenderTarget::unbind()
{
    // Unbind FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
//----------------------------------------------------------------------------
void RenderTarget::clear()
{
    // Clear colour and depth buffer
    // **TODO** different clear colours
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
}   // namespace AntWorld
}   // namespace BoBRobotics
