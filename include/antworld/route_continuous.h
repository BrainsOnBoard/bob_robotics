#pragma once

// BoB robotics includes
#include "common/pose.h"

// OpenGL includes
#include <GL/glew.h>

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <array>
#include <random>
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
    //! Load route from matlab-format binary file
    void load(const std::string &filename);

    //! Generate levy walk
    template<typename T>
    void generateLevyWalk(const Vector2<meter_t> &origin, size_t numSteps, meter_t stepSizeMean, meter_t stepSizeScale, T &rng)
    {
        using namespace units::math;

        // Create distributions
        std::uniform_real_distribution<double> angleDistribution(0.0, 360.0);
        std::cauchy_distribution<double> stepSizeDistrbution(stepSizeMean.value(), stepSizeScale.value());

        // Clear and allocate waypoints
        m_Waypoints.clear();
        m_Waypoints.resize(numSteps + 1ul);

        // Add origin
        m_Waypoints[0][0] = (GLfloat)origin.x().value();
        m_Waypoints[0][1] = (GLfloat)origin.y().value();
        // Loop through steps
        Vector2<meter_t> point = origin;
        for(size_t i = 1; i <= numSteps; i++) {
            // Sample angle and step size
            const degree_t angle{angleDistribution(rng)};
            const meter_t stepSize{stepSizeDistrbution(rng)};

            // Update point and add to waypoints
            point.x() += stepSize * sin(angle);
            point.y() += stepSize * cos(angle);
            m_Waypoints[i][0] = (GLfloat)point.x().value();
            m_Waypoints[i][1] = (GLfloat)point.y().value();
        }

        // Rebuild route
        rebuildRoute();
    }

    void render(const Pose2<meter_t, degree_t> &pose, meter_t height = meter_t{0.1}) const;

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
    // Private methods
    //------------------------------------------------------------------------
    //! Rebuild route from list of waypoints
    void rebuildRoute();

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
