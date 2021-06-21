#pragma once

// OpenGL includes
#include <GL/glew.h>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTarget
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
class RenderTarget
{
public:
    RenderTarget(GLsizei width, GLsizei height);
    ~RenderTarget();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void bind();
    void unbind();
    void clear();

    GLuint getTexture() const{ return m_Texture; }
    GLsizei getWidth() const{ return m_Width; }
    GLsizei getHeight() const{ return m_Height; }
    GLuint getFBO() const{ return m_FBO; }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    const GLsizei m_Width;
    const GLsizei m_Height;

    GLuint m_FBO;
    GLuint m_Texture;
    GLuint m_DepthBuffer;
};
}   // namespace AntWorld
}   // namespace BoBRobotics