#pragma once

// Standard C++ includes
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// OpenGL includes
#include <GL/glew.h>

// Antworld includes
#include "render_mesh.h"
#include "render_target.h"
#include "world.h"

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics
{
namespace AntWorld
{
using namespace units::literals;

//----------------------------------------------------------------------------
// RendererBase
//----------------------------------------------------------------------------
//! Helper class which combines a world with a rendermesh to allow ant views of world to be rendered to screen
class RendererBase
{
protected:
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;

public:
    virtual ~RendererBase()
    {
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void renderFirstPersonView(meter_t x, meter_t y, meter_t z,
                               degree_t yaw, degree_t pitch, degree_t roll,
                               GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);
    void renderFirstPersonView(meter_t x, meter_t y, meter_t z,
                               degree_t yaw, degree_t pitch, degree_t roll,
                               RenderTarget &renderTarget, bool bind = true, bool clear = true);
    void renderTopDownView(GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);
    void renderTopDownView(RenderTarget &renderTarget, bool bind = true, bool clear = true);

    World &getWorld(){ return m_World; }
    const World &getWorld() const{ return m_World; }

protected:
    RendererBase(GLdouble nearClip = 0.001, GLdouble farClip = 1000.0);

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual void renderPanoramicGeometry();
    virtual void renderFirstPersonGeometry();
    virtual void renderTopDownGeometry();

    //------------------------------------------------------------------------
    // Protected API
    //------------------------------------------------------------------------
    void generateCubeFaceLookAtMatrices(GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ,
                                        GLfloat (&cubeFaceLookAtMatrices)[6][16]);
    void applyFrame(meter_t x, meter_t y, meter_t z,
                    degree_t yaw, degree_t pitch, degree_t roll);

    GLdouble getNearClip() const{ return m_NearClip; }
    GLdouble getFarClip() const{ return m_FarClip; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    World m_World;

    const GLdouble m_NearClip;
    const GLdouble m_FarClip;
};

//----------------------------------------------------------------------------
// Renderer
//----------------------------------------------------------------------------
class Renderer : public RendererBase
{
public:
    template<class... Ts>
    Renderer(GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0, Ts &&... args)
    :   RendererBase(nearClip, farClip), m_RenderMesh(false, std::forward<Ts>(args)...),
        m_RenderTargetCubemap(cubemapSize)
    {
        // Pre-generate lookat matrices to point at cubemap faces
        generateCubeFaceLookAtMatrices(0.0, 0.0, 0.0,
                                       m_CubeFaceLookAtMatrices);
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void renderPanoramicView(meter_t x, meter_t y, meter_t z,
                             degree_t yaw, degree_t pitch, degree_t roll,
                             GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight,
                             GLuint drawFBO = 0);
    void renderPanoramicView(meter_t x, meter_t y, meter_t z,
                             degree_t yaw, degree_t pitch, degree_t roll,
                             RenderTarget &renderTarget, bool bind = true, bool clear = true);


private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLfloat m_CubeFaceLookAtMatrices[6][16];

    RenderMeshSpherical m_RenderMesh;
    RenderTargetCubemap m_RenderTargetCubemap;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
