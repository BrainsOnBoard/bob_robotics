// BoB robotics includes
#include "antworld/route_continuous.h"
#include "antworld/common.h"
#include "common/macros.h"
#include "plog/Log.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <tuple>

using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::math;

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RouteContinuous
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RouteContinuous::RouteContinuous(float arrowLength, unsigned int maxRouteEntries)
    : m_WaypointsVAO(0), m_WaypointsPositionVBO(0), m_WaypointsColourVBO(0),
    m_RouteVAO(0), m_RoutePositionVBO(0), m_RouteColourVBO(0), m_RouteNumPoints(0), m_RouteMaxPoints(maxRouteEntries),
    m_OverlayVAO(0), m_OverlayPositionVBO(0), m_OverlayColoursVBO(0), m_MinBound{0_m, 0_m}, m_MaxBound{0_m, 0_m}
{
    const GLfloat arrowPositions[] = {
        0.0f, 0.0f,
        0.0f, arrowLength,
    };

    const GLfloat arrowColours[] = {
        1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f
    };

    // Create a vertex array object to bind everything together
    glGenVertexArrays(1, &m_OverlayVAO);

    // Generate vertex buffer objects for positions and colours
    glGenBuffers(1, &m_OverlayPositionVBO);
    glGenBuffers(1, &m_OverlayColoursVBO);

    // Bind vertex array
    glBindVertexArray(m_OverlayVAO);

    // Bind and upload positions buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_OverlayPositionVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 2, arrowPositions, GL_STATIC_DRAW);

    // Set vertex pointer to stride over angles and enable client state in VAO
    glVertexPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

     // Bind and upload colours buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_OverlayColoursVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 6, arrowColours, GL_STATIC_DRAW);

    // Set colour pointer and enable client state in VAO
    glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);

    // Create a vertex array object to bind everything together
    glGenVertexArrays(1, &m_RouteVAO);

    // Generate vertex buffer objects for positions and colours
    glGenBuffers(1, &m_RoutePositionVBO);
    glGenBuffers(1, &m_RouteColourVBO);

    // Bind vertex array
    glBindVertexArray(m_RouteVAO);

    // Bind and upload positions buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RoutePositionVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * maxRouteEntries, nullptr, GL_DYNAMIC_DRAW);

    // Set vertex pointer to stride over angles and enable client state in VAO
    glVertexPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

     // Bind and upload colours buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RouteColourVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(uint8_t) * 2 * maxRouteEntries, nullptr, GL_DYNAMIC_DRAW);

    // Set colour pointer and enable client state in VAO
    glColorPointer(3, GL_UNSIGNED_BYTE, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);

    // Unbind
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
RouteContinuous::RouteContinuous(float arrowLength, unsigned int maxRouteEntries, const std::string &filename)
    : RouteContinuous(arrowLength, maxRouteEntries)
{
    load(filename);
}
//----------------------------------------------------------------------------
RouteContinuous::~RouteContinuous()
{
    // Delete waypoint objects
    glDeleteBuffers(1, &m_WaypointsPositionVBO);
    glDeleteVertexArrays(1, &m_WaypointsColourVBO);
    glDeleteVertexArrays(1, &m_WaypointsVAO);

    // Delete route objects
    glDeleteBuffers(1, &m_RoutePositionVBO);
    glDeleteVertexArrays(1, &m_RouteColourVBO);
    glDeleteVertexArrays(1, &m_RouteVAO);

    // Delete overlay objects
    glDeleteBuffers(1, &m_OverlayPositionVBO);
    glDeleteBuffers(1, &m_OverlayColoursVBO);
    glDeleteVertexArrays(1, &m_OverlayVAO);
}
//----------------------------------------------------------------------------
void RouteContinuous::load(const std::string &filename)
{
    // Open file for binary IO
    std::ifstream input(filename, std::ios::binary);
    if(!input.good()) {
        throw std::runtime_error("Cannot open route file: " + filename);
    }

    // Seek to end of file, get size and rewind
    input.seekg(0, std::ios_base::end);
    const auto numPoints = static_cast<size_t>(input.tellg()) / (sizeof(double) * 3);
    input.seekg(0);
    LOG_INFO << "Route has " << numPoints << " points";

    // Clear and allocate waypoints
    m_Waypoints.clear();
    m_Waypoints.resize(numPoints);

    // Loop through components(X and Y, ignoring heading)
    for(unsigned int c = 0; c < 2; c++) {
        // Loop through points on path
        for(unsigned int i = 0; i < numPoints; i++) {
            // Read point component
            double pointPosition;
            input.read(reinterpret_cast<char*>(&pointPosition), sizeof(pointPosition));

            // Scale to metres and insert into route
            m_Waypoints[i][c] = static_cast<GLfloat>(pointPosition / 100.0);
        }
    }

    // Rebuild route
    rebuildRoute();
}
//----------------------------------------------------------------------------
void RouteContinuous::render(const Pose2<meter_t, degree_t> &pose, meter_t height) const
{
    // Bind route VAO
    glBindVertexArray(m_WaypointsVAO);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, static_cast<GLfloat>(height.value()));
    glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(m_Waypoints.size()));

    // If there are any route points, bind
    if(m_RouteNumPoints > 0) {
        glBindVertexArray(m_RouteVAO);
        glDrawArrays(GL_LINE_STRIP, 0, m_RouteNumPoints);
    }

    glBindVertexArray(m_OverlayVAO);

    glTranslatef(static_cast<GLfloat>(pose.x().value()), static_cast<GLfloat>(pose.y().value()), 0.01f);
    glRotatef(static_cast<GLfloat>(-pose.yaw().value()), 0.0f, 0.0f, 1.0f);
    glDrawArrays(GL_LINES, 0, 2);
    glPopMatrix();

    // Unbind VAOs
    glBindVertexArray(0);
}
//----------------------------------------------------------------------------
bool RouteContinuous::atDestination(const Vector2<meter_t> &position, meter_t threshold) const
{
    // If route's empty, there is no destination so return false
    if(m_Waypoints.empty()) {
        return false;
    }
    // Otherwise return true if
    else {
        return distance(m_Waypoints.back(), position.x(), position.y()) < threshold;
    }
}
//----------------------------------------------------------------------------
std::tuple<meter_t, meter_t> RouteContinuous::getDistanceToRoute(const Vector2<meter_t> &position) const
{
    // Loop through segments
    meter_t minimumDistance = std::numeric_limits<meter_t>::max();
    size_t nearestWaypoint = std::numeric_limits<size_t>::max();

    for (unsigned int s = 0; s < m_Waypoints.size(); s++) {
        const meter_t distanceToWaypoint = distance(m_Waypoints[s], position.x(), position.y());

        // If this is closer than current minimum, update minimum and nearest waypoint
        if(distanceToWaypoint < minimumDistance) {
            minimumDistance = distanceToWaypoint;
            nearestWaypoint = s;
        }
    }

    // Return the minimum distance to the path and the segment in which this occured
    return std::make_tuple(minimumDistance, m_CumulativeDistance.at(nearestWaypoint));
}
//----------------------------------------------------------------------------
Pose2<meter_t, degree_t> RouteContinuous::getPose(meter_t distance) const
{
    // Clamp distance at 0
    distance = max(0_m, distance);

    // Find waypoint AFTER distance
    auto nextWaypointIter = std::upper_bound(m_CumulativeDistance.cbegin(), m_CumulativeDistance.cend(), distance);

    // Get its index (if we are beyond end of route use last waypoint)
    const size_t nextWaypointIndex = (nextWaypointIter == m_CumulativeDistance.cend())
        ? (m_CumulativeDistance.size() - 1)
        : (size_t)std::distance(m_CumulativeDistance.cbegin(), nextWaypointIter);

    // Get next waypoint's distance and position
    const meter_t nextWaypointDistance = m_CumulativeDistance[nextWaypointIndex];
    const auto &nextWaypoint = m_Waypoints[nextWaypointIndex];

    // Get index, distance and position of PREVIOUS waypoint
    const size_t prevWaypointIndex = nextWaypointIndex - 1;
    const meter_t prevWaypointDistance = m_CumulativeDistance[prevWaypointIndex];
    const auto &prevWaypoint = m_Waypoints[prevWaypointIndex];

    // Determine what proportion of segment distance is at
    const double proportion = std::min(1.0, std::max(0.0,
        ((distance - prevWaypointDistance) / (nextWaypointDistance - prevWaypointDistance)).value()));

    // Interpolate position
    const meter_t x{ prevWaypoint[0] + proportion * (nextWaypoint[0] - prevWaypoint[0]) };
    const meter_t y{ prevWaypoint[1] + proportion * (nextWaypoint[1] - prevWaypoint[1]) };

    // Return position
    return Pose2<meter_t, degree_t>(x, y, 90_deg + m_Headings[prevWaypointIndex]);
}
//----------------------------------------------------------------------------
void RouteContinuous::setWaypointFamiliarity(size_t pos, double familiarity)
{
    // Convert familiarity to a grayscale colour
    const uint8_t intensity = (uint8_t)std::min(255.0, std::max(0.0, std::round(255.0 * familiarity)));
    const uint8_t colour[3] = {intensity, intensity, intensity};

    // Update this positions colour in colour buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_WaypointsColourVBO);
    glBufferSubData(GL_ARRAY_BUFFER, pos * sizeof(uint8_t) * 3, sizeof(uint8_t) * 3, colour);
}
//----------------------------------------------------------------------------
void RouteContinuous::addPoint(const Vector2<meter_t> &position, bool error)
{
    constexpr static uint8_t errorColour[3] = {0xFF, 0, 0};
    constexpr static uint8_t correctColour[3] = {0, 0xFF, 0};

    const float positionRaw[2] = {(float)position.x().value(), (float)position.y().value()};

    // Check we haven't run out of buffer space
    BOB_ASSERT(m_RouteNumPoints < m_RouteMaxPoints);

    // Update this vertex's colour in colour buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RouteColourVBO);
    glBufferSubData(GL_ARRAY_BUFFER, m_RouteNumPoints * sizeof(uint8_t) * 3,
                    sizeof(uint8_t) * 3, error ? errorColour : correctColour);

    // Update this vertex's position in position buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RoutePositionVBO);
    glBufferSubData(GL_ARRAY_BUFFER, m_RouteNumPoints * sizeof(float) * 2,
                    sizeof(float) * 2, positionRaw);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m_RouteNumPoints++;
}
//----------------------------------------------------------------------------
void RouteContinuous::rebuildRoute()
{
    // Calculate minimum and maximum bounds in X and Y
    const auto minMaxX = std::minmax_element(m_Waypoints.cbegin(), m_Waypoints.cend(),
                                             [](const std::array<GLfloat, 2> &a, const std::array<GLfloat, 2> &b)
                                             {
                                                 return (a[0] < b[0]);
                                             });
    const auto minMaxY = std::minmax_element(m_Waypoints.cbegin(), m_Waypoints.cend(),
                                             [](const std::array<GLfloat, 2> &a, const std::array<GLfloat, 2> &b)
                                             {
                                                 return (a[1] < b[1]);
                                             });
    m_MinBound[0] = meter_t{(*minMaxX.first)[0]};
    m_MaxBound[0] = meter_t{(*minMaxX.second)[0]};
    m_MinBound[1] = meter_t{(*minMaxY.first)[1]};
    m_MaxBound[1] = meter_t{(*minMaxY.second)[1]};
    LOG_INFO << "Route min: (" << m_MinBound[0] << ", " << m_MinBound[1] << ")";
    LOG_INFO << "Route max: (" << m_MaxBound[0] << ", " << m_MaxBound[1] << ")";

    // Clear existing  headings and cumulative distances
    m_Headings.clear();
    m_CumulativeDistance.clear();

    // Reserve headings
    const size_t numSegments = m_Waypoints.size() - 1;
    m_Headings.reserve(numSegments);

    // Reserve array of cumulative distances
    m_CumulativeDistance.reserve(m_Waypoints.size());
    m_CumulativeDistance.push_back(0_m);

    // Loop through route segments
    for(unsigned int i = 0; i < numSegments; i++) {
        // Get waypoints at start and end of segment
        const auto &segmentStart = m_Waypoints[i];
        const auto &segmentEnd = m_Waypoints[i + 1];

        // Add segment heading to array
        m_Headings.push_back(atan2(units::length::meter_t(segmentStart[1] - segmentEnd[1]),
                                   units::length::meter_t(segmentEnd[0] - segmentStart[0])));

        // Calculate segment length and
        const meter_t segmentLength(distance(segmentStart, segmentEnd));
        m_CumulativeDistance.push_back(m_CumulativeDistance.back() + segmentLength);
    }

    // Create a vertex array object to bind everything together
    if(m_WaypointsVAO == 0) {
        glGenVertexArrays(1, &m_WaypointsVAO);
    }

    // Generate vertex buffer objects for positions and colours
    if(m_WaypointsPositionVBO == 0) {
        glGenBuffers(1, &m_WaypointsPositionVBO);
    }

    if(m_WaypointsColourVBO == 0) {
        glGenBuffers(1, &m_WaypointsColourVBO);
    }

    // Bind vertex array
    glBindVertexArray(m_WaypointsVAO);

    // Bind and upload positions buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_WaypointsPositionVBO);
    glBufferData(GL_ARRAY_BUFFER, m_Waypoints.size() * sizeof(GLfloat) * 2, m_Waypoints.data(), GL_STATIC_DRAW);

    // Set vertex pointer and enable client state in VAO
    glVertexPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

    {
        // Bind and upload zeros to colour buffer
        std::vector<uint8_t> colours(m_Waypoints.size() * 3, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_WaypointsColourVBO);
        glBufferData(GL_ARRAY_BUFFER, m_Waypoints.size() * sizeof(uint8_t) * 3, colours.data(), GL_DYNAMIC_DRAW);

        // Set colour pointer and enable client state in VAO
        glColorPointer(3, GL_UNSIGNED_BYTE, 0, BUFFER_OFFSET(0));
        glEnableClientState(GL_COLOR_ARRAY);
    }

    // Unbind VAOs
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
}   // namespace AntWorld
}   // namespace BoBRobotics
