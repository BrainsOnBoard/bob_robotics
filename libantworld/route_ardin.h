#pragma once

// Standard C++ includes
#include <array>
#include <set>
#include <string>
#include <vector>

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Third-party includes
#include "../third_party/units.h"

using namespace units::literals;
using namespace units::angle;

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RouteArdin
//----------------------------------------------------------------------------
// Class for reading ant routes exported by Matlab, performing 'straightening'
// Process from original matlab code and rendering them in ant world
namespace BoBRobotics
{
namespace AntWorld
{
class RouteArdin
{
public:
    RouteArdin(float arrowLength, unsigned int maxRouteEntries);
    RouteArdin(float arrowLength, unsigned int maxRouteEntries, const std::string &filename, bool realign = true);
    ~RouteArdin();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool load(const std::string &filename, bool realign = true);
    void render(float antX, float antY, float antHeading) const;

    bool atDestination(float x, float y, float threshold) const;
    std::tuple<float, size_t> getDistanceToRoute(float x, float y) const;
    void setWaypointFamiliarity(size_t pos, double familiarity);
    void addPoint(float x, float y, bool error);

    size_t size() const{ return m_Waypoints.size(); }

    //------------------------------------------------------------------------
    // Operators
    //------------------------------------------------------------------------
    std::tuple<float, float, degree_t> operator[](size_t waypoint) const;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLuint m_WaypointsVAO;
    GLuint m_WaypointsPositionVBO;
    GLuint m_WaypointsColourVBO;

    GLuint m_RouteVAO;
    GLuint m_RoutePositionVBO;
    GLuint m_RouteColourVBO;
    unsigned int m_RouteNumPoints;

    std::vector<std::array<float, 2>> m_Waypoints;
    std::vector<degree_t> m_Headings;
    std::set<size_t> m_TrainedSnapshots;

    GLuint m_OverlayVAO;
    GLuint m_OverlayPositionVBO;
    GLuint m_OverlayColoursVBO;
};
}   // namespace AntWorld
}   // namespace BoBRobotics