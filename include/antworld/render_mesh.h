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
using namespace units::literals;

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

    //------------------------------------------------------------------------
    // RenderMesh::Border
    //------------------------------------------------------------------------
    //! Helper class for defining the bounds of render mesh FOV
    class Border
    {
        using degree_t = units::angle::degree_t;
    public:
        Border(degree_t horizontalFOV, degree_t verticalFOV, degree_t centreAzimuth, degree_t centreElevation);
        Border(const std::string &eyeBorderFilename, bool duplicateLeftRight);

        //----------------------------------------------------------------------------
        // Public API
        //----------------------------------------------------------------------------
        degree_t getMinAzimuth() const{ return m_MinAzimuth; }
        degree_t getMaxAzimuth() const{ return m_MaxAzimuth; }

        degree_t getMinElevation() const{ return m_MinElevation; }
        degree_t getMaxElevation() const{ return m_MaxElevation; }

        bool isInEye(degree_t azimuth, degree_t elevation) const;

    private:
        //----------------------------------------------------------------------------
        // Members
        //----------------------------------------------------------------------------
        const bool m_DuplicateLeftRight;

        degree_t m_MinAzimuth;
        degree_t m_MaxAzimuth;

        degree_t m_MinElevation;
        degree_t m_MaxElevation;

        std::vector<std::tuple<degree_t, degree_t>> m_EyeBorderVertices;
    };

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
    using degree_t = units::angle::degree_t;
public:
    RenderMeshSpherical(units::angle::degree_t horizontalFOV = 296_deg, units::angle::degree_t verticalFOV = 75_deg,
                        units::angle::degree_t startElevation  = 15_deg,
                        unsigned int numHorizontalSegments = 40, unsigned int numVerticalSegments = 10);
    RenderMeshSpherical(const std::string &eyeBorderFilename, bool duplicateLeftRight = true,
                        unsigned int numHorizontalSegments = 40, unsigned int numVerticalSegments = 10);

protected:
    RenderMeshSpherical(const Border &border, unsigned int numHorizontalSegments, unsigned int numVerticalSegments);
};

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshHexagonal
//----------------------------------------------------------------------------
//! Class for sampling cubemap across a range of hexagonal coordinates
class RenderMeshHexagonal : public RenderMesh
{
    using degree_t = units::angle::degree_t;
public:
    RenderMeshHexagonal(const std::string &eyeBorderFilename, units::angle::degree_t interommatidiaAngle);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    unsigned int getNumHorizontalHexes() const{ return m_NumHorizontalHexes; }
    unsigned int getNumVerticalHexes() const{ return m_NumVerticalHexes; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    unsigned int m_NumHorizontalHexes;
    unsigned int m_NumVerticalHexes;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
