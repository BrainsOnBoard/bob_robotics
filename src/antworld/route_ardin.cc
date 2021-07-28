#include "antworld/route_ardin.h"

// Standard C++ includes
#include <fstream>
#include <iostream>
#include <limits>
#include <tuple>

// PLOG includes
#include "plog/Log.h"

// Libantworld includes
#include "antworld/common.h"

using namespace units::angle;
using namespace units::length;
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RouteArdin
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RouteArdin::RouteArdin(float arrowLength, unsigned int maxRouteEntries)
    : m_WaypointsVAO(0), m_WaypointsPositionVBO(0), m_WaypointsColourVBO(0),
    m_RouteVAO(0), m_RoutePositionVBO(0), m_RouteColourVBO(0), m_RouteNumPoints(0),
    m_OverlayVAO(0), m_OverlayPositionVBO(0), m_OverlayColoursVBO(0),
    m_MinBound{0_m, 0_m}, m_MaxBound{0_m, 0_m}
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
RouteArdin::RouteArdin(float arrowLength, unsigned int maxRouteEntries,
                       const std::string &filename, bool realign)
    : RouteArdin(arrowLength, maxRouteEntries)
{
    load(filename, realign);
}
//----------------------------------------------------------------------------
RouteArdin::~RouteArdin()
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
void RouteArdin::load(const std::string &filename, bool realign)
{
    // Clear existing waypoints and headings
    m_Waypoints.clear();
    m_Headings.clear();
    m_Colours.clear();
    m_TrainedSnapshots.clear();

    // Open file for binary IO
    std::ifstream input(filename, std::ios::binary);
    input.exceptions(std::ios::badbit | std::ios::failbit);

    // Seek to end of file, get size and rewind
    input.seekg(0, std::ios_base::end);
    const auto numPoints = static_cast<size_t>(input.tellg()) / (sizeof(double) * 3);
    input.seekg(0);
    LOG_INFO << "Route has " << numPoints << " points";

    {
        // Loop through components(X and Y, ignoring heading)
        std::vector<std::array<float, 2>> fullRoute(numPoints);
        for(unsigned int c = 0; c < 2; c++) {
            // Loop through points on path
            for(unsigned int i = 0; i < numPoints; i++) {
                // Read point component
                double pointPosition;
                input.read(reinterpret_cast<char*>(&pointPosition), sizeof(double));

                // Convert to float, scale to metres and insert into route
                fullRoute[i][c] = (float)pointPosition * (1.0f / 100.0f);
            }
        }

        // Reserve correctly sized vector for waypoints
        m_Waypoints.reserve((numPoints / 10) + 1);

        // Loop through every 10 path points and add waypoint
        for(unsigned int i = 0; i < numPoints; i += 10)
        {
            m_Waypoints.push_back(fullRoute[i]);
        }
    }

    // Reserve headings
    const size_t numSegments = m_Waypoints.size() - 1;
    m_Headings.reserve(numSegments);

    // Loop through route segments
    for(unsigned int i = 0; i < numSegments; i++) {
        // Get waypoints at start and end of segment
        const auto &segmentStart = m_Waypoints[i];
        const auto &segmentEnd = m_Waypoints[i + 1];

        // Calculate segment heading (NB: using unit.h's atan2, not cmath's)
        const degree_t heading = units::math::atan2(units::length::meter_t(segmentStart[1] - segmentEnd[1]),
                                                    units::length::meter_t(segmentEnd[0] - segmentStart[0]));

        // Round to nearest whole number and add to headings array
        m_Headings.push_back(units::math::round(heading * 0.5) * 2.0);
    }

    // Loop through waypoints other than first
    if(realign) {
        for(unsigned int i = 1; i < m_Waypoints.size(); i++)
        {
            // Get previous and current waypoiny
            const auto &prevWaypoint = m_Waypoints[i - 1];
            auto &waypoint = m_Waypoints[i];

            // Convert the segment heading back to radians
            const radian_t heading = m_Headings[i - 1];

            // Realign segment to this angle
            waypoint[0] = prevWaypoint[0] + (0.1 * units::math::cos(heading));
            waypoint[1] = prevWaypoint[1] - (0.1 * units::math::sin(heading));
        }
    }

    // Initialise bounds to limits of underlying data types
    std::fill_n(&m_MinBound[0], 2, std::numeric_limits<meter_t>::max());
    std::fill_n(&m_MaxBound[0], 2, std::numeric_limits<meter_t>::min());

    // Calculate bounds from waypoints
    for(const auto &w : m_Waypoints) {
        for(unsigned int c = 0; c < 2; c++) {
            m_MinBound[c] = units::math::min(m_MinBound[c], meter_t(w[c]));
            m_MaxBound[c] = units::math::max(m_MaxBound[c], meter_t(w[c]));
        }
    }

    LOG_INFO << "Min: (" << m_MinBound[0] << ", " << m_MinBound[1] << ")";
    LOG_INFO << "Max: (" << m_MaxBound[0] << ", " << m_MaxBound[1] << ")";

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

    // Resize and zero colours
    m_Colours.resize(m_Waypoints.size(), {0, 0, 0});

    // Bind and upload colour buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_WaypointsColourVBO);
    glBufferData(GL_ARRAY_BUFFER, m_Colours.size() * sizeof(uint8_t) * 3, m_Colours.data(), GL_DYNAMIC_DRAW);

    // Set colour pointer and enable client state in VAO
    glColorPointer(3, GL_UNSIGNED_BYTE, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);

    // Unbind VAOs
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
void RouteArdin::render(const Pose2<meter_t, degree_t> &pose, meter_t height) const
{
    // Bind route VAO
    glBindVertexArray(m_WaypointsVAO);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, static_cast<GLfloat>(height.value()));
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_Waypoints.size()));

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
bool RouteArdin::atDestination(const Vector2<meter_t> &position, meter_t threshold) const
{
    // If route's empty, there is no destination so return false
    if(m_Waypoints.empty()) {
        return false;
    }
    // Otherwise return true if
    else {
        return (distance(m_Waypoints.back(), position.x(), position.y()) < threshold);
    }
}
//----------------------------------------------------------------------------
std::tuple<meter_t, size_t> RouteArdin::getDistanceToRoute(const Vector2<meter_t> &position) const
{
    // Loop through segments
    meter_t minimumDistance = std::numeric_limits<meter_t>::max();
    size_t nearestWaypoint;
    for(unsigned int s = 0; s < m_Waypoints.size(); s++)
    {
        const meter_t distanceToWaypoint = distance(m_Waypoints[s], position.x(), position.y());

        // If this is closer than current minimum, update minimum and nearest waypoint
        if(distanceToWaypoint < minimumDistance) {
            minimumDistance = distanceToWaypoint;
            nearestWaypoint = s;
        }
    }

    // Return the minimum distance to the path and the segment in which this occured
    return std::make_tuple(minimumDistance, nearestWaypoint);
}
//----------------------------------------------------------------------------
void RouteArdin::setWaypointFamiliarity(size_t pos, double familiarity)
{
    // Convert familiarity to a grayscale colour
    const uint8_t intensity = (uint8_t)std::min(255.0, std::max(0.0, std::round(255.0 * familiarity)));
    m_Colours[pos] = {intensity, intensity, intensity};

    // Update this positions colour in colour buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_WaypointsColourVBO);
    glBufferSubData(GL_ARRAY_BUFFER, pos * sizeof(uint8_t) * 3, sizeof(uint8_t) * 3, &m_Colours[pos]);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
void RouteArdin::highlightWaypointRange(size_t begin, size_t end)
{
    // Define colours
    const std::array<uint8_t, 3> black{0, 0, 0};
    const std::array<uint8_t, 3> white{255, 255, 255};

    // Constrain end within waypoints
    end = std::min(end, m_Waypoints.size());

    // Fill black before begin
    std::fill(m_Colours.begin(), m_Colours.begin() + begin, black);

    // Fill white between begin and end
    std::fill(m_Colours.begin() + begin, m_Colours.begin() + end, white);

    // Fill black after end
    std::fill(m_Colours.begin() + end, m_Colours.end(), black);

    // Re-upload colours
    glBindBuffer(GL_ARRAY_BUFFER, m_WaypointsColourVBO);
    glBufferData(GL_ARRAY_BUFFER, m_Colours.size() * sizeof(uint8_t) * 3, m_Colours.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
void RouteArdin::addPoint(const Vector2<meter_t> &position, bool error)
{
    const static uint8_t errorColour[3] = {0xFF, 0, 0};
    const static uint8_t correctColour[3] = {0, 0xFF, 0};

    const float positionRaw[2] = { (float)position.x().value(), (float)position.y().value() };

    // Update this positions colour in colour buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RouteColourVBO);
    glBufferSubData(GL_ARRAY_BUFFER, m_RouteNumPoints * sizeof(uint8_t) * 3,
                    sizeof(uint8_t) * 3, error ? errorColour : correctColour);

    // Update this positions colour in colour buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_RoutePositionVBO);
    glBufferSubData(GL_ARRAY_BUFFER, m_RouteNumPoints * sizeof(float) * 2,
                    sizeof(float) * 2, positionRaw);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m_RouteNumPoints++;
}
//----------------------------------------------------------------------------
Pose2<meter_t, degree_t> RouteArdin::operator[](size_t waypoint) const
{
    const meter_t x{ m_Waypoints[waypoint][0] };
    const meter_t y{ m_Waypoints[waypoint][1] };

    // If this isn't the last waypoint, return the heading of the segment from this waypoint
    if(waypoint < m_Headings.size()) {
        return {x, y, 90_deg + m_Headings[waypoint]};
    }
    else {
        return {x, y, 0_deg};
    }
}
}   // namespace AntWorld
}   // namespace BoBRobotics
