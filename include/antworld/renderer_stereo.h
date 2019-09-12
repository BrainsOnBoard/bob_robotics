#pragma once

// Antworld includes
#include "renderer.h"
#include "render_target.h"


//------------------------------------------------------------------------
// BoBRobotics::AntWorld::RendererStereo
//------------------------------------------------------------------------
//! Renderer which provides
namespace BoBRobotics
{
namespace AntWorld
{
class RendererStereo : public RendererBase
{
public:
    template<class... Ts>
    RendererStereo(GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0,
                   GLdouble eyeSeperation = 0.001, Ts &&... args)
    :   RendererBase(nearClip, farClip), m_RenderMeshLeft(false, std::forward<Ts>(args)...),
        m_RenderMeshRight(true, std::forward<Ts>(args)...), m_RenderTargetCubemapLeft(cubemapSize),
        m_RenderTargetCubemapRight(cubemapSize)
    {
        // Pre-generate lookat matrices to point at cubemap faces
        generateCubeFaceLookAtMatrices(eyeSeperation / 2.0, 0.0, 0.0,
                                       m_CubeFaceLookAtMatricesLeft);
        generateCubeFaceLookAtMatrices(eyeSeperation / -2.0, 0.0, 0.0,
                                       m_CubeFaceLookAtMatricesLeft);
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
    // Private methods
    //------------------------------------------------------------------------
    void renderCubemap(const GLfloat (&agentMatrix)[16], const GLfloat (&cubeFaceLookAtMatrices)[6][16],
                       RenderTargetCubemap &renderTarget);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLfloat m_CubeFaceLookAtMatricesLeft[6][16];
    GLfloat m_CubeFaceLookAtMatricesRight[6][16];

    RenderMeshSpherical m_RenderMeshLeft;
    RenderMeshSpherical m_RenderMeshRight;
    RenderTargetCubemap m_RenderTargetCubemapLeft;
    RenderTargetCubemap m_RenderTargetCubemapRight;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
