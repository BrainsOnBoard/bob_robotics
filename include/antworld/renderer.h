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
//! Standard renderer: renders panoramic world views to a single cubemap and renders these using a single RenderMesh
class Renderer : public RendererBase
{
public:
    Renderer(std::unique_ptr<RenderMesh> renderMesh,
             GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0);

    // Legacy constructor
    Renderer(GLsizei cubemapSize = 256, double nearClip = 0.001, double farClip = 1000.0,
             degree_t horizontalFOV = 296_deg, degree_t verticalFOV = 75_deg);

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

    RenderMesh *getRenderMesh(){ return m_RenderMesh.get(); }
    const RenderMesh *getRenderMesh() const{ return m_RenderMesh.get(); }

    //------------------------------------------------------------------------
    // Static API
    //------------------------------------------------------------------------
    template<class... Ts>
    static std::unique_ptr<Renderer> createSpherical(GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0,
                                                     Ts &&... renderMeshArgs)
    {
        return std::make_unique<Renderer>(std::make_unique<RenderMeshSpherical>(false, std::forward<Ts>(renderMeshArgs)...),
                                          cubemapSize, nearClip, farClip);
    }

    template<class... Ts>
    static std::unique_ptr<Renderer> createHexagonal(GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0,
                                                     Ts &&... renderMeshArgs)
    {
        return std::make_unique<Renderer>(std::make_unique<RenderMeshHexagonal>(false, std::forward<Ts>(renderMeshArgs)...),
                                          cubemapSize, nearClip, farClip);
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLfloat m_CubeFaceLookAtMatrices[6][16];

    std::unique_ptr<RenderMesh> m_RenderMesh;
    RenderTargetCubemap m_RenderTargetCubemap;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
