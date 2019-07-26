#pragma once

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Third-party includes
#include "../third_party/units.h"

// Lib antworld includes
#include "surface.h"

namespace BoBRobotics
{
namespace AntWorld
{
class Render;
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMesh
//----------------------------------------------------------------------------
//! Class for generating piece of curved geometry used for rendering cubemap to screen
class RenderMesh
{
public:
    virtual ~RenderMesh();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void render() const;

protected:
    RenderMesh() : m_NumIndices(0), m_IBO(0)
    {
    }

    Surface &getSurface(){ return m_Surface; }

private:
    //-----------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    Surface m_Surface;
};

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshCylinder
//----------------------------------------------------------------------------
//! Class for generating piece of curved geometry used for rendering cubemap to screen
class RenderMeshCylinder : public RenderMesh
{
public:
    RenderMeshCylinder(units::angle::degree_t horizontalFOV, units::angle::degree_t verticalFOV, units::angle::degree_t startLongitude,
                       unsigned int numHorizontalSegments, unsigned int numVerticalSegments);
};
}   // namespace AntWorld
}   // namespace BoBRobotics
