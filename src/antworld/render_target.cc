// BoB robotics includes
#include "antworld/render_target.h"

// Standard C++ includes
#include <stdexcept>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetBase
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RenderTargetBase::~RenderTargetBase()
{
    glDeleteFramebuffers(1, &m_FBO);
}
//----------------------------------------------------------------------------
void RenderTargetBase::bind()
{
    // Bind the cubemap FBO for offscreen rendering
    glBindFramebuffer(GL_FRAMEBUFFER, getFBO());
}
//----------------------------------------------------------------------------
void RenderTargetBase::unbind(GLuint drawFBO)
{
    // Unbind FBO
    glBindFramebuffer(GL_FRAMEBUFFER, drawFBO);
}
//----------------------------------------------------------------------------
void RenderTargetBase::clear()
{
    // Clear colour and depth buffer
    // **TODO** different clear colours
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
//----------------------------------------------------------------------------
RenderTargetBase::RenderTargetBase(GLsizei width, GLsizei height)
:   m_Width(width), m_Height(height)
{
    // Create FBO for rendering to cubemap
    glGenFramebuffers(1, &m_FBO);
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTarget
//----------------------------------------------------------------------------
RenderTarget::RenderTarget(GLsizei width, GLsizei height)
:   RenderTargetBase(width, height)
{
    // Bind FBO
    bind();

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
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetCubemap
//----------------------------------------------------------------------------
RenderTargetCubemap::RenderTargetCubemap(GLsizei size)
:   RenderTargetBase(size, size)
{
    // Bind FBO
    bind();

    // Create cubemap and bind
    glGenTextures(1, &m_CubemapTexture);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_CubemapTexture);

    // Create textures for all faces of cubemap
    // **NOTE** even though we don't need top and bottom faces we still need to create them or rendering fails
    // **TODO** it would be better to use native format
    for(unsigned int t = 0; t < 6; t++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + t, 0, GL_RGB,
                     size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    }
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    // Create depth render buffer
    glGenRenderbuffers(1, &m_DepthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_DepthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, size, size);

    // Attach depth buffer to frame buffer
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_DepthBuffer);

    // Check frame buffer is created correctly
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("Frame buffer not complete");
    }

    // Unbind cube map and frame buffer
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
//----------------------------------------------------------------------------
RenderTargetCubemap::~RenderTargetCubemap()
{
    glDeleteRenderbuffers(1, &m_DepthBuffer);
    glDeleteTextures(1, &m_CubemapTexture);
}
//------------------------------------------------------------------------
void RenderTargetCubemap::attachCubemapFace(GLenum face)
{
    // Attach correct frame buffer face to frame buffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, face, m_CubemapTexture, 0);

}
}   // namespace AntWorld
}   // namespace BoBRobotics
