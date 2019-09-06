#pragma once

// OpenGL includes
#include <GL/glew.h>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetBase
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
class RenderTargetBase
{
public:
    virtual ~RenderTargetBase();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void bind();
    void unbind(GLuint drawFBO = 0);
    void clear();

    GLsizei getWidth() const{ return m_Width; }
    GLsizei getHeight() const{ return m_Height; }
    GLuint getFBO() const{ return m_FBO; }

protected:
    RenderTargetBase(GLsizei width, GLsizei height);

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    const GLsizei m_Width;
    const GLsizei m_Height;

    GLuint m_FBO;
};

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetBase
//----------------------------------------------------------------------------
class RenderTarget : public RenderTargetBase
{
public:
    RenderTarget(GLsizei width, GLsizei height);
    virtual ~RenderTarget();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    GLuint getTexture() const{ return m_Texture; }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    GLuint m_Texture;
    GLuint m_DepthBuffer;
};

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetBase
//----------------------------------------------------------------------------
class RenderTargetCubemap : public RenderTargetBase
{
public:
    RenderTargetCubemap(GLsizei size);
    virtual ~RenderTargetCubemap();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void attachCubemapFace(GLenum face);

    GLuint getCubemapTexture() const{ return m_CubemapTexture; }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    GLuint m_CubemapTexture;
    GLuint m_DepthBuffer;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
