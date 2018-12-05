#pragma once

// Standard C++ includes
#include <string>

// OpenGL includes
#include <GL/glew.h>

// Antworld includes
#include "render_mesh.h"
#include "world.h"

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics
{
namespace AntWorld
{
using namespace units::literals;

//----------------------------------------------------------------------------
// Renderer
//----------------------------------------------------------------------------
//! Helper class which combines a world with a rendermesh to allow ant views of world to be rendered to screen
class Renderer
{
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;

public:
    Renderer(GLsizei cubemapSize = 256, double nearClip = 0.001, double farClip = 1000.0,
             degree_t horizontalFOV = 296_deg, degree_t verticalFOV = 75_deg);
    ~Renderer();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void renderPanoramicView(meter_t x, meter_t y, meter_t z,
                             degree_t yaw, degree_t pitch, degree_t roll,
                             GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);
    void renderFirstPersonView(meter_t x, meter_t y, meter_t z,
                               degree_t yaw, degree_t pitch, degree_t roll,
                               GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);
    void renderTopDownView(GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);

    World &getWorld(){ return m_World; }
    const World &getWorld() const{ return m_World; }
private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void generateCubeFaceLookAtMatrices();
    void applyFrame(meter_t x, meter_t y, meter_t z,
                    degree_t yaw, degree_t pitch, degree_t roll);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    World m_World;
    RenderMesh m_RenderMesh;

    GLuint m_CubemapTexture;
    GLuint m_FBO;
    GLuint m_DepthBuffer;
    GLfloat m_CubeFaceLookAtMatrices[6][16];

    const GLsizei m_CubemapSize;
    const double m_NearClip;
    const double m_FarClip;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
