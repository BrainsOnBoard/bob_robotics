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
    void render(const Vector2<meter_t> &position, degree_t antHeading, meter_t height = meter_t{0.1}) const;

    bool atDestination(const Vector2<meter_t> &position, meter_t threshold) const;
    std::tuple<meter_t, meter_t> getDistanceToRoute(const Vector2<meter_t> &position) const;
    Pose2<meter_t, degree_t> getPose(meter_t distance) const;

    void setWaypointFamiliarity(size_t pos, double familiarity);
    void addPoint(const Vector2<meter_t> &position, bool error);
    meter_t getLength() const{ return m_CumulativeDistance.back(); }

    const Vector2<meter_t> &getMinBound()
    {
        return m_MinBound;
    }

    const Vector2<meter_t> &getMaxBound()
    {
        return m_MaxBound;
    }

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

    Vector2<meter_t> m_MinBound;
    Vector2<meter_t> m_MaxBound;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
