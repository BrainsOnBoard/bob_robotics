#pragma once

// Standard C++ includes
#include <vector>
#include <tuple>

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Third-party includes
#include "third_party/units.h"

// Lib antworld includes
#include "antworld/surface.h"

namespace BoBRobotics
{
namespace AntWorld
{
class Render;
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMesh
//----------------------------------------------------------------------------
//! Class for generating geometry on which to render cubemap to screen
class RenderMesh
{
public:
    virtual ~RenderMesh()
    {
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void render() const;

protected:
    RenderMesh()
    {
    }

    Surface &getSurface(){ return m_Surface; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    Surface m_Surface;
};

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshSpherical
//----------------------------------------------------------------------------
//! Class for sampling cubemap across a range of spherical coordiates defined by horizontal and vertical FOV,
//! converting these to cartesian coordinates and rendering them to grid with desired number of horizontal and vertical segments
class RenderMeshSpherical : public RenderMesh
{
public:
    RenderMeshSpherical(units::angle::degree_t horizontalFOV, units::angle::degree_t verticalFOV, units::angle::degree_t startLongitude,
                        unsigned int numHorizontalSegments, unsigned int numVerticalSegments);
};

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshHexagonal
//----------------------------------------------------------------------------
//! Class for sampling cubemap across a range of hexagonal coordinates
class RenderMeshHexagonal : public RenderMesh
{
public:
    RenderMeshHexagonal(const std::string &eyeBorderFilename, units::angle::degree_t interommatidiaAngle);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    unsigned int getNumHorizontalHexes() const{ return m_NumHorizontalHexes; }
    unsigned int getNumVerticalHexes() const{ return m_NumVerticalHexes; }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    std::vector<std::tuple<float, float>> loadEyeBorder(const std::string &eyeBorderFilename) const;

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    unsigned int m_NumHorizontalHexes;
    unsigned int m_NumVerticalHexes;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
