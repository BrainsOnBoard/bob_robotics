#pragma once

// BoB robotics includes
#include "common/pose.h"

// Third-party includes
#include "third_party/units.h"

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Standard C++ includes
#include <array>
#include <set>
#include <string>
#include <vector>

namespace BoBRobotics
{
namespace AntWorld
{
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RouteArdin
//----------------------------------------------------------------------------
/*!
 * \brief Class for reading ant routes exported by Matlab, performing 'straightening'
 *
 * Processed from original Matlab code and rendering them in ant world
 */
class RouteArdin
{
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;

public:
    RouteArdin(float arrowLength, unsigned int maxRouteEntries);
    RouteArdin(float arrowLength, unsigned int maxRouteEntries, const std::string &filename, bool realign = true);
    ~RouteArdin();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void load(const std::string &filename, bool realign = true);
    void render(const Pose2<meter_t, degree_t> &pose, meter_t height = meter_t{0.1}) const;

    bool atDestination(const Vector2<meter_t> &position, meter_t threshold) const;
    std::tuple<meter_t, size_t> getDistanceToRoute(const Vector2<meter_t> &position) const;
    void setWaypointFamiliarity(size_t pos, double familiarity);
    void highlightWaypointRange(size_t begin, size_t end);
    void addPoint(const Vector2<meter_t> &position, bool error);

    const Vector2<meter_t> &getMinBound()
    {
        return m_MinBound;
    }

    const Vector2<meter_t> &getMaxBound()
    {
        return m_MaxBound;
    }

    size_t size() const{ return m_Waypoints.size(); }

    //------------------------------------------------------------------------
    // Operators
    //------------------------------------------------------------------------
    Pose2<meter_t, degree_t> operator[](size_t waypoint) const;

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
    std::vector<std::array<uint8_t, 3>> m_Colours;
    std::vector<degree_t> m_Headings;
    std::set<size_t> m_TrainedSnapshots;

    GLuint m_OverlayVAO;
    GLuint m_OverlayPositionVBO;
    GLuint m_OverlayColoursVBO;

    Vector2<meter_t> m_MinBound;
    Vector2<meter_t> m_MaxBound;
};
}   // namespace AntWorld
}   // namespace BoBRobotics

