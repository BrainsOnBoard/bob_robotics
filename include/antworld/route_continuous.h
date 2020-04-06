#pragma once

// BoB robotics includes
#include "common/pose.h"

// OpenGL includes
#include <GL/glew.h>

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <array>
#include <string>
#include <vector>

namespace BoBRobotics
{
namespace AntWorld
{
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RouteContinuous
//----------------------------------------------------------------------------
//! Class for reading ant routes exported by Matlab and rendering them in ant world
class RouteContinuous
{
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;

public:
    RouteContinuous(float arrowLength, unsigned int maxRouteEntries);
    RouteContinuous(float arrowLength, unsigned int maxRouteEntries, const std::string &filename);
    ~RouteContinuous();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void load(const std::string &filename);
    void render(meter_t antX, meter_t antY, degree_t antHeading) const;

    bool atDestination(meter_t x, meter_t y, meter_t threshold) const;
    std::tuple<meter_t, size_t> getDistanceToRoute(meter_t x, meter_t y) const;
    Pose2<meter_t, degree_t> getPose(meter_t distance) const;

    void setWaypointFamiliarity(size_t pos, double familiarity);
    void addPoint(meter_t x, meter_t y, bool error);
    meter_t getLength() const{ return m_CumulativeDistance.back(); }

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

    std::vector<std::array<GLfloat, 2>> m_Waypoints;
    std::vector<degree_t> m_Headings;
    std::vector<meter_t> m_CumulativeDistance;
    GLuint m_OverlayVAO;
    GLuint m_OverlayPositionVBO;
    GLuint m_OverlayColoursVBO;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
