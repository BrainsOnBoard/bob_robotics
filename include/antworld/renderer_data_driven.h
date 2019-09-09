#pragma once

// Antworld includes
#include "renderer.h"
#include "render_target.h"


//------------------------------------------------------------------------
// BoBRobotics::AntWorld::RendererDataDriven
//------------------------------------------------------------------------
//! Renderer which provides
namespace BoBRobotics
{
namespace AntWorld
{
class RendererDataDriven : public RendererBase
{
public:
    RendererDataDriven(const std::string &eyeBorderFilename, units::angle::degree_t interommatidiaAngle,
                       GLsizei cubemapSize = 256, GLdouble nearClip = 0.001, GLdouble farClip = 1000.0);

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
    // Private methods
    std::vector<std::tuple<float, float>> loadEyeBorder(const std::string &eyeBorderFilename) const;

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLfloat m_CubeFaceLookAtMatrices[6][16];

    RenderMeshHexagonal m_RenderMesh;
    RenderTargetCubemap m_RenderTargetCubemap;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
