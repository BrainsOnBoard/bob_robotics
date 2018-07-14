#pragma once

// Standard C++ includes
#include <array>
#include <string>
#include <vector>

// OpenGL includes
#include <GL/glew.h>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RouteContinuous
//----------------------------------------------------------------------------
// Class for reading ant routes exported by Matlab and rendering them in ant world
namespace BoBRobotics
{
namespace AntWorld
{
class RouteContinuous
{
public:
    RouteContinuous(float arrowLength, unsigned int maxRouteEntries);
    RouteContinuous(float arrowLength, unsigned int maxRouteEntries, const std::string &filename);
    ~RouteContinuous();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool load(const std::string &filename);
    void render(float antX, float antY, float antHeading) const;

    bool atDestination(float x, float y, float threshold) const;
    std::tuple<float, size_t> getDistanceToRoute(float x, float y) const;
    std::tuple<float, float, float> getPosition(float distance) const;

    void setWaypointFamiliarity(size_t pos, double familiarity);
    void addPoint(float x, float y, bool error);
    float getLength() const{ return m_CumulativeDistance.back(); }

    size_t size() const{ return m_Waypoints.size(); }

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
    const unsigned int m_RouteMaxPoints;

    std::vector<std::array<float, 2>> m_Waypoints;
    std::vector<float> m_HeadingDegrees;
    std::vector<float> m_CumulativeDistance;
    GLuint m_OverlayVAO;
    GLuint m_OverlayPositionVBO;
    GLuint m_OverlayColoursVBO;
};
}   // namespace AntWorld
}   // namespace BoBRobotics