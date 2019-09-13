#pragma once

// Antworld includes
#include "renderer.h"
#include "render_target.h"

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::RendererStereo
//------------------------------------------------------------------------
//! Stereo renderer: renders panoramic world views to left and right cubemaps and renders these using a pair of RenderMeshes
namespace BoBRobotics
{
namespace AntWorld
{
class RendererStereo : public RendererBase
{
public:
    RendererStereo(std::unique_ptr<RenderMesh> renderMeshLeft, std::unique_ptr<RenderMesh> renderMeshRight,
                   GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0, GLdouble eyeSeperation = 0.001);

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

    RenderMesh *getRenderMeshLeft(){ return m_RenderMeshLeft.get(); }
    const RenderMesh *getRenderMesh() const{ return m_RenderMeshLeft.get(); }

    RenderMesh *getRenderMeshRight(){ return m_RenderMeshRight.get(); }
    const RenderMesh *getRenderMeshRight() const{ return m_RenderMeshRight.get(); }


    //------------------------------------------------------------------------
    // Static API
    //------------------------------------------------------------------------
    template<class... Ts>
    static std::unique_ptr<RendererStereo> createSpherical(GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0,
                                                           GLdouble eyeSeperation = 0.001, Ts &&... renderMeshArgs)
    {
        return std::make_unique<RendererStereo>(std::make_unique<RenderMeshSpherical>(true, std::forward<Ts>(renderMeshArgs)...),
                                                std::make_unique<RenderMeshSpherical>(false, std::forward<Ts>(renderMeshArgs)...),
                                                cubemapSize, nearClip, farClip, eyeSeperation);
    }

    template<class... Ts>
    static std::unique_ptr<RendererStereo> createHexagonal(GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0,
                                                           GLdouble eyeSeperation = 0.001, Ts &&... renderMeshArgs)
    {
        return std::make_unique<RendererStereo>(std::make_unique<RenderMeshHexagonal>(true, std::forward<Ts>(renderMeshArgs)...),
                                                std::make_unique<RenderMeshHexagonal>(false, std::forward<Ts>(renderMeshArgs)...),
                                                cubemapSize, nearClip, farClip, eyeSeperation);
    }

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

    std::unique_ptr<RenderMesh> m_RenderMeshLeft;
    std::unique_ptr<RenderMesh> m_RenderMeshRight;
    RenderTargetCubemap m_RenderTargetCubemapLeft;
    RenderTargetCubemap m_RenderTargetCubemapRight;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
