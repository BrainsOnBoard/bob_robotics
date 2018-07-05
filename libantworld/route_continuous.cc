#include "route_continuous.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <tuple>

// Standard C++ includes
#include <cassert>
#include <cmath>

// Libantworld includes
#include "common.h"

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
float sqr(float x)
{
    return (x * x);
}
//----------------------------------------------------------------------------
float distanceSquared(float x1, float y1, float x2, float y2)
{
    return sqr(x2 - x1) + sqr(y2 - y1);
}
}   // Anonymous namespace

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
    m_OverlayVAO(0), m_OverlayPositionVBO(0), m_OverlayColoursVBO(0)
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
}
//----------------------------------------------------------------------------
RouteContinuous::RouteContinuous(float arrowLength, unsigned int maxRouteEntries, const std::string &filename)
    : RouteContinuous(arrowLength, maxRouteEntries)
{
    if(!load(filename)) {
        throw std::runtime_error("Cannot load route");
    }
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
bool RouteContinuous::load(const std::string &filename)
{
    // Open file for binary IO
    std::ifstream input(filename, std::ios::binary);
    if(!input.good()) {
        std::cerr << "Cannot open route file:" << filename << std::endl;
        return false;
    }

    // Seek to end of file, get size and rewind
    input.seekg(0, std::ios_base::end);
    const std::streampos numPoints = input.tellg() / (sizeof(double) * 3);
    input.seekg(0);
    std::cout << "Route has " << numPoints << " points" << std::endl;

    // Allocate path points
    m_Waypoints.resize(numPoints);

    // Loop through components(X and Y, ignoring heading)
    float min[2] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    float max[2] = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
    for(unsigned int c = 0; c < 2; c++) {
        // Loop through points on path
        for(unsigned int i = 0; i < numPoints; i++) {
            // Read point component
            double pointPosition;
            input.read(reinterpret_cast<char*>(&pointPosition), sizeof(double));

            // Convert to float, scale to metres and insert into route
            m_Waypoints[i][c] = (float)pointPosition * (1.0f / 100.0f);

            // Update min and max
            min[c] = std::min(min[c], m_Waypoints[i][c]);
            max[c] = std::max(max[c], m_Waypoints[i][c]);
        }
    }

    std::cout << "X range = (" << min[0] << ", " << max[0] << "), y range = (" << min[1] << ", " << max[1] << ")" << std::endl;
    // Reserve headings
    const unsigned int numSegments = m_Waypoints.size() - 1;
    m_HeadingDegrees.reserve(numSegments);

    // Reserve array of cumulative distances
    m_CumulativeDistance.reserve(m_Waypoints.size());
    m_CumulativeDistance.push_back(0.0f);

    // Loop through route segments
    for(unsigned int i = 0; i < numSegments; i++) {
        // Get waypoints at start and end of segment
        const auto &segmentStart = m_Waypoints[i];
        const auto &segmentEnd = m_Waypoints[i + 1];

        // Add segment heading to array
        m_HeadingDegrees.push_back(radiansToDegrees * atan2(segmentStart[1] - segmentEnd[1],
                                                            segmentEnd[0] - segmentStart[0]));

        // Calculate segment length and
        const float segmentLength = sqrt(distanceSquared(segmentStart[0], segmentStart[1],
                                                         segmentEnd[0], segmentEnd[1]));
        m_CumulativeDistance.push_back(m_CumulativeDistance.back() + segmentLength);
    }

    // Create a vertex array object to bind everything together
    glGenVertexArrays(1, &m_WaypointsVAO);

    // Generate vertex buffer objects for positions and colours
    glGenBuffers(1, &m_WaypointsPositionVBO);
    glGenBuffers(1, &m_WaypointsColourVBO);

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
    return true;
}
//----------------------------------------------------------------------------
void RouteContinuous::render(float antX, float antY, float antHeading) const
{
    // Bind route VAO
    glBindVertexArray(m_WaypointsVAO);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.1f);
    glDrawArrays(GL_POINTS, 0, m_Waypoints.size());

    // If there are any route points, bind
    if(m_RouteNumPoints > 0) {
        glBindVertexArray(m_RouteVAO);

        glDrawArrays(GL_LINE_STRIP, 0, m_RouteNumPoints);
    }

    glBindVertexArray(m_OverlayVAO);

    glTranslatef(antX, antY, 0.1f);
    glRotatef(-antHeading, 0.0f, 0.0f, 1.0f);
    glDrawArrays(GL_LINES, 0, 2);
    glPopMatrix();

}
//----------------------------------------------------------------------------
bool RouteContinuous::atDestination(float x, float y, float threshold) const
{
    // If route's empty, there is no destination so return false
    if(m_Waypoints.empty()) {
        return false;
    }
    // Otherwise return true if
    else {
        return (distanceSquared(x, y, m_Waypoints.back()[0], m_Waypoints.back()[1]) < sqr(threshold));
    }
}
//----------------------------------------------------------------------------
std::tuple<float, size_t> RouteContinuous::getDistanceToRoute(float x, float y) const
{
    // Loop through segments
    float minimumDistanceSquared = std::numeric_limits<float>::max();
    size_t nearestWaypoint;
    for(unsigned int s = 0; s < m_Waypoints.size(); s++)
    {
        const float distanceToWaypointSquared = distanceSquared(x, y, m_Waypoints[s][0], m_Waypoints[s][1]);

        // If this is closer than current minimum, update minimum and nearest waypoint
        if(distanceToWaypointSquared < minimumDistanceSquared) {
            minimumDistanceSquared = distanceToWaypointSquared;
            nearestWaypoint = s;
        }
    }

    // Return the minimum distance to the path and the segment in which this occured
    return std::make_tuple(sqrt(minimumDistanceSquared), nearestWaypoint);
}
//----------------------------------------------------------------------------
std::tuple<float, float, float> RouteContinuous::getPosition(float distance) const
{
    // Clamp distance at 0
    distance = std::max(0.0f, distance);

    // Find waypoint AFTER distance
    auto nextWaypointIter = std::upper_bound(m_CumulativeDistance.cbegin(), m_CumulativeDistance.cend(), distance);

    // Get it's index (if we are beyond end of route use last waypoint
    const size_t nextWaypointIndex = (nextWaypointIter == m_CumulativeDistance.cend())
        ? (m_CumulativeDistance.size() - 1)
        : (size_t)std::distance(m_CumulativeDistance.cbegin(), nextWaypointIter);

    // Get next waypoint's distance and position
    const float nextWaypointDistance = m_CumulativeDistance[nextWaypointIndex];
    const float nextWaypointX = m_Waypoints[nextWaypointIndex][0];
    const float nextWaypointY = m_Waypoints[nextWaypointIndex][1];

    // Get index, distance and position of PREVIOUS waypoint
    const size_t prevWaypointIndex = nextWaypointIndex - 1;
    const float prevWaypointDistance = m_CumulativeDistance[prevWaypointIndex];
    const float prevWaypointX = m_Waypoints[prevWaypointIndex][0];
    const float prevWaypointY = m_Waypoints[prevWaypointIndex][1];

    // Determine what proportion of segment distance is at
    const float proportion = std::min(1.0f, std::max(0.0f, (distance - prevWaypointDistance) / (nextWaypointDistance - prevWaypointDistance)));

    // Interpolate position
    const float x = prevWaypointX + proportion * (nextWaypointX - prevWaypointX);
    const float y = prevWaypointY + proportion * (nextWaypointY - prevWaypointY);

    // Return position
    return std::make_tuple(x, y, 90.0f + m_HeadingDegrees[prevWaypointIndex]);
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
void RouteContinuous::addPoint(float x, float y, bool error)
{
    constexpr static uint8_t errorColour[3] = {0xFF, 0, 0};
    constexpr static uint8_t correctColour[3] = {0, 0xFF, 0};

    const float position[2] = {x, y};

    // Check we haven't run out of buffer space
    assert(m_RouteNumPoints < m_RouteMaxPoints);

    // Update this vertex's colour in colour buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RouteColourVBO);
    glBufferSubData(GL_ARRAY_BUFFER, m_RouteNumPoints * sizeof(uint8_t) * 3,
                    sizeof(uint8_t) * 3, error ? errorColour : correctColour);

    // Update this vertex's position in position buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RoutePositionVBO);
    glBufferSubData(GL_ARRAY_BUFFER, m_RouteNumPoints * sizeof(float) * 2,
                    sizeof(float) * 2, position);

    m_RouteNumPoints++;
}
}   // namespace AntWorld
}   // namespace BoBRobotics