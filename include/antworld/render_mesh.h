#pragma once

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics
{
namespace AntWorld
{
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMesh
//----------------------------------------------------------------------------
//! Class for generating piece of curved geometry used for rendering cubemap to screen
class RenderMesh
{
public:
    RenderMesh();
    RenderMesh(units::angle::degree_t horizontalFOV, units::angle::degree_t verticalFOV, units::angle::degree_t startLongitude,
               unsigned int numHorizontalSegments, unsigned int numVerticalSegments);
    ~RenderMesh();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void render() const;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLuint m_VAO;
    GLuint m_PositionVBO;
    GLuint m_TextureCoordsVBO;
    GLuint m_IBO;
    unsigned int m_NumIndices;
};
}   // namespace AntWorld
}   // namespace BoBRobotics