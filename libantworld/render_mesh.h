#pragma once

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Third-party includes
#include "../third_party/units.h"

using namespace units::literals;
using namespace units::angle;

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMesh
//----------------------------------------------------------------------------
// Class for generating piece of curved geometry used for rendering cubemap to screen
namespace BoBRobotics
{
namespace AntWorld
{
class RenderMesh
{
public:
    RenderMesh();
    RenderMesh(degree_t horizontalFOV, degree_t verticalFOV, degree_t startLongitude,
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