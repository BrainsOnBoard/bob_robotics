#include "route_continuous.h"

// BoB robotics includes
#include "../common/assert.h"
#include "../third_party/csv.h"
#include "common.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <tuple>

using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::math;

namespace
{
// Transform from

struct Transform
{
    // Translation
    const meter_t tx;
    const meter_t ty;
    const meter_t tz;

    // Scale
    const double s;

    // Rotation
    const radian_t rx;
    const radian_t ry;
    const radian_t rz;
};

struct Ellipsoid
{
    // Major and minor axis
    const meter_t a;
    const meter_t b;

    // Eccentricity
    const double f;
};

// WGS84 datum
struct WGS84
{
    static const Ellipsoid ellipsoid;
};
const Ellipsoid WGS84::ellipsoid{6378137_m, 6356752.314245_m, 1.0 / 298.257223563};

// OSGB-36 datum
struct OSGB36
{
    static const Transform transform;
    static const Ellipsoid ellipsoid;
};
const Transform OSGB36::transform{-446.448_m, 125.157_m, -542.060_m,  20.4894, -0.1502_arcsec, -0.2470_arcsec, -0.8421_arcsec};
const Ellipsoid OSGB36::ellipsoid{6377563.396_m, 6356256.909_m, 1.0 / 299.3249646};

// An OS coordinate consisting of an 'easting' and 'northing' distance
typedef std::tuple<meter_t, meter_t> OSCoordinate;

// A latitude and longitude in the given space
template<typename S>
struct LatLon
{
    typedef S Space;

    const degree_t lat;
    const degree_t lon;
};

// A cartesian coordinate in the given space
template<typename S>
struct Cartesian
{
    typedef S Space;

    const meter_t x;
    const meter_t y;
    const meter_t z;
};


// Convert latitude and longitude to cartesian
// Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence)
template<typename Space>
Cartesian<Space> latLonToCartesian(const LatLon<Space> &latLon)
{
    const meter_t h = 0.0_m; // height above ellipsoid - not currently used
    const double sinPhi = sin(latLon.lat);
    const double cosPhi = cos(latLon.lat);
    const double sinLambda = sin(latLon.lon);
    const double cosLambda = cos(latLon.lon);

    const double eSq = 2.0 * Space::ellipsoid.f - Space::ellipsoid.f * Space::ellipsoid.f;  // 1st eccentricity squared ≡ (a²-b²)/a²
    const meter_t nu = Space::ellipsoid.a / std::sqrt(1.0 - eSq * sinPhi * sinPhi);         // radius of curvature in prime vertical

    return Cartesian<Space>{(nu + h) * cosPhi * cosLambda,
                            (nu + h) * cosPhi * sinLambda,
                            (nu * (1.0 - eSq) + h) * sinPhi};
}

// Transform cartesian coordinates from WGS84 into desired space
// Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence)
template<typename Space>
Cartesian<Space> transformCartesian(const Cartesian<WGS84> &c)
{
    const double s1 = Space::transform.s / 1E6 + 1.0;            // scale: normalise parts-per-million to (s+1)

    const double rx = Space::transform.rx.value();
    const double ry = Space::transform.ry.value();
    const double rz = Space::transform.rz.value();

    // apply transform
    return Cartesian<Space>{Space::transform.tx + c.x * s1 - c.y * rz + c.z * ry,
                            Space::transform.ty + c.x * rz + c.y * s1 - c.z * rx,
                            Space::transform.tz - c.x * ry + c.y * rx + c.z * s1};
}

// Convert cartesian coordinates back to latitude and longitude
// Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence)
template<typename Space>
LatLon<Space> cartesianToLatLon(const Cartesian<Space> &c)
{
    const double e2 = 2.0 * Space::ellipsoid.f - Space::ellipsoid.f *Space::ellipsoid.f;    // 1st eccentricity squared ≡ (a²-b²)/a²
    const double epsilon2 = e2 / (1.0 - e2);                                                // 2nd eccentricity squared ≡ (a²-b²)/b²
    const meter_t p = sqrt(c.x * c.x + c.y * c.y);  // distance from minor axis
    const meter_t r = sqrt(p * p + c.z * c.z);      // polar radius

    // parametric latitude (Bowring eqn 17, replacing tanβ = z·a / p·b)
    const double tanBeta = (Space::ellipsoid.b * c.z) / (Space::ellipsoid.a * p) * (1.0 + epsilon2 * Space::ellipsoid.b / r);
    const double sinBeta = tanBeta / std::sqrt(1.0 + tanBeta * tanBeta);
    const double cosBeta = sinBeta / tanBeta;

    // geodetic latitude (Bowring eqn 18: tanφ = z+ε²bsin³β / p−e²cos³β)
    const degree_t phi = std::isnan(cosBeta) ? 0.0_rad : atan2(c.z + epsilon2 * Space::ellipsoid.b * sinBeta * sinBeta * sinBeta,
                                                               p - e2 * Space::ellipsoid.a * cosBeta * cosBeta * cosBeta);

    // longitude
    const degree_t lambda = atan2(c.y, c.x);

    // height above ellipsoid (Bowring eqn 7) [not currently used]
    /*const double sinPhi = sin(phi), cosPhi = cos(phi);
    const meter_t nu = std::get<0>(ellipsoid) / std::sqrt(1.0 - e2 * sinPhi * sinPhi); // length of the normal terminated by the minor axis
    var h = p*cosφ + cartesian[2]*sinφ - (osgb36EllipseA*osgb36EllipseA/ν);*/

    return LatLon<Space>{phi, lambda};
}

// Convert latitude and longitude in the OSGB36 space to OS grid coordinates
// Ported from Javascript Ordnance Survey Grid Reference functions (c) Chris Veness 2005-2017 (MIT Licence)
OSCoordinate latLonToOS(const LatLon<OSGB36> &latLon)
{
    const meter_t a = 6377563.396_m, b = 6356256.909_m;                 // Airy 1830 major & minor semi-axes
    constexpr double f0 = 0.9996012717;                                 // NatGrid scale factor on central meridian;
    const radian_t phi0 = 49.0_deg, lambda0 = -2.0_deg;                 // NatGrid true origin is 49°N,2°W
    const meter_t n0 = -100000.0_m, e0 = 400000.0_m;                    // northing & easting of true origin, metres
    const double e2 = 1.0 - (b * b) / (a * a);                      // eccentricity squared
    const double n = (a - b) / (a + b), n2 = n * n, n3 = n * n * n; // n, n², n³

    const radian_t phi = latLon.lat;
    const radian_t lambda = latLon.lon;

    const double sinPhi = sin(phi);
    const double cosPhi = cos(phi);
    const double tanPhi = tan(phi);
    const double cos3Phi = cosPhi * cosPhi * cosPhi;
    const double cos5Phi = cos3Phi * cosPhi * cosPhi;
    const double tan2Phi = tanPhi * tanPhi;
    const double tan4Phi = tan2Phi * tan2Phi;


    const meter_t nu = a * f0 / std::sqrt(1.0 - e2 * sinPhi * sinPhi);                      // nu = transverse radius of curvature
    const meter_t rho = a * f0 * (1.0 - e2) / std::pow(1.0 - e2 * sinPhi * sinPhi, 1.5);    // rho = meridional radius of curvature
    const double eta2 = nu / rho - 1.0;                                                     // eta = ?

    const double mA = (1.0 + n + (5.0 / 4.0) * n2 + (5.0 / 4.0) * n3) * (phi - phi0).value();
    const double mB = (3.0 * n + 3.0 * n * n + (21.0 / 8.0) * n3) * sin(phi - phi0) * cos(phi + phi0);
    const double mC = ((15.0 / 8.0) * n2 + (15.0 / 8.0) * n3) * sin(2.0 * (phi - phi0)) * cos(2.0 * (phi + phi0));
    const double mD = (35.0 / 24.0) * n3 * sin(3.0 * (phi - phi0)) * cos(3.0 * (phi + phi0));
    const meter_t m = b * f0 * (mA - mB + mC - mD);                                                                  // meridional arc

    const meter_t i = m + n0;
    const meter_t ii = (nu / 2.0) * sinPhi * cosPhi;
    const meter_t iii = (nu / 24.0) * sinPhi * cos3Phi * (5.0 - tan2Phi + 9.0 * eta2);
    const meter_t iiia = (nu / 720.0) * sinPhi * cos5Phi * (61.0 - 58.0 * tan2Phi + tan4Phi);
    const meter_t iv = nu * cosPhi;
    const meter_t v = (nu / 6.0) * cos3Phi * (nu / rho - tan2Phi);
    const meter_t vi = (nu / 120.0) * cos5Phi * (5.0 - 18.0 * tan2Phi + tan4Phi+ 14.0 * eta2 - 58.0 * tan2Phi * eta2);


    const double deltaLambda = (lambda - lambda0).value();
    const double deltaLambda2 = deltaLambda * deltaLambda;
    const double deltaLambda3 = deltaLambda2 * deltaLambda;
    const double deltaLambda4 = deltaLambda3 * deltaLambda;
    const double deltaLambda5 = deltaLambda4 * deltaLambda;
    const double deltaLambda6 = deltaLambda5 * deltaLambda;

    return std::make_tuple(e0 + (iv * deltaLambda) + (v * deltaLambda3) + (vi * deltaLambda5),
                           i + (ii * deltaLambda2) + (iii * deltaLambda4) + (iiia * deltaLambda6));
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
    std::cout << "Route has " << numPoints << " points" << std::endl;

    // Allocate path points
    m_Waypoints.resize(numPoints);

    // Loop through components(X and Y, ignoring heading)
    GLfloat min[2] = {std::numeric_limits<GLfloat>::max(), std::numeric_limits<GLfloat>::max()};
    GLfloat max[2] = {std::numeric_limits<GLfloat>::lowest(), std::numeric_limits<GLfloat>::lowest()};
    for(unsigned int c = 0; c < 2; c++) {
        // Loop through points on path
        for(unsigned int i = 0; i < numPoints; i++) {
            // Read point component
            double pointPosition;
            input.read(reinterpret_cast<char*>(&pointPosition), sizeof(pointPosition));

            // Scale to metres and insert into route
            m_Waypoints[i][c] = static_cast<GLfloat>(pointPosition / 100.0);

            // Update min and max
            min[c] = std::min(min[c], m_Waypoints[i][c]);
            max[c] = std::max(max[c], m_Waypoints[i][c]);
        }
    }

    std::cout << "X range = (" << min[0] << ", " << max[0] << "), y range = (" << min[1] << ", " << max[1] << ")" << std::endl;
    createGeometry();
}
//----------------------------------------------------------------------------
void RouteContinuous::loadRadarCSV(const std::string &filename)
{
    // Create reader to efficiently read the three columns we care about
    io::CSVReader<3> in(filename);

    // Read header, ignoring A LOT of extra  columns!
    in.read_header(io::ignore_extra_column, "TimeSinceStart", "DataGPSLat", "DataGPSLong");

    // Read CSV rows
    double timeSinceStart, gpsLatRaw, gpsLongRaw;
    GLfloat min[2] = {std::numeric_limits<GLfloat>::max(), std::numeric_limits<GLfloat>::max()};
    GLfloat max[2] = {std::numeric_limits<GLfloat>::lowest(), std::numeric_limits<GLfloat>::lowest()};
    while(in.read_row(timeSinceStart, gpsLatRaw, gpsLongRaw)){
        const degree_t phiDegree{gpsLatRaw};
        const degree_t lambdaDegree{gpsLongRaw};

        // 1) GPS coordinates are based on the WGS84 ellisoid, first convert to Cartesian
        LatLon<WGS84> wgs84LatLon{phiDegree, lambdaDegree};
        const auto wgs84Cartesian = latLonToCartesian(wgs84LatLon);

        // 2) Transform Cartesian into OSGB36 space
        const auto osGB36Cartesian = transformCartesian<OSGB36>(wgs84Cartesian);

        // 3) Convert cartesian back to latitude and longitude (now on OSGB36 ellipsoid)
        const auto osGBLatLon = cartesianToLatLon(osGB36Cartesian);

        // 4) Finally, convert latitude and longitude to OS coordinate
        const auto osCoord = latLonToOS(osGBLatLon);

        std::cout << timeSinceStart << ", (" << phiDegree << ", " << lambdaDegree << ") = (" << std::get<0>(osCoord) << ", " << std::get<1>(osCoord) << ")" << std::endl;

        m_Waypoints.emplace_back();
        m_Waypoints.back()[0] = (GLfloat)std::get<0>(osCoord);
        m_Waypoints.back()[1] = (GLfloat)std::get<1>(osCoord);

        // Update min and max
        min[0] = std::min(min[0], m_Waypoints.back()[0]);
        min[1] = std::min(min[1], m_Waypoints.back()[1]);
        max[0] = std::max(max[0], m_Waypoints.back()[0]);
        max[1] = std::max(max[1], m_Waypoints.back()[1]);
    }
    std::cout << "X range = (" << min[0] << ", " << max[0] << "), y range = (" << min[1] << ", " << max[1] << ")" << std::endl;
    createGeometry();
}
//----------------------------------------------------------------------------
void RouteContinuous::renderWaypoints(GLfloat height) const
{
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, height);
    glDrawArrays(GL_POINTS, 0, m_Waypoints.size());
    glPopMatrix();
}
//----------------------------------------------------------------------------
void RouteContinuous::render(meter_t antX, meter_t antY, degree_t antHeading) const
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

    glTranslatef(antX.value(), antY.value(), 0.1);
    glRotatef(-antHeading.value(), 0.0, 0.0, 1.0);
    glDrawArrays(GL_LINES, 0, 2);
    glPopMatrix();

}
//----------------------------------------------------------------------------
bool RouteContinuous::atDestination(meter_t x, meter_t y, meter_t threshold) const
{
    // If route's empty, there is no destination so return false
    if(m_Waypoints.empty()) {
        return false;
    }
    // Otherwise return true if
    else {
        return distance(m_Waypoints.back(), x, y) < threshold;
    }
}
//----------------------------------------------------------------------------
std::tuple<meter_t, size_t> RouteContinuous::getDistanceToRoute(meter_t x, meter_t y) const
{
    // Loop through segments
    meter_t minimumDistance = std::numeric_limits<meter_t>::max();
    size_t nearestWaypoint;
    for(unsigned int s = 0; s < m_Waypoints.size(); s++)
    {
        const meter_t distanceToWaypoint = distance(m_Waypoints[s], x, y);

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
std::tuple<meter_t, meter_t, degree_t> RouteContinuous::getPosition(meter_t distance) const
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
    return std::make_tuple(x, y, 90_deg + m_Headings[prevWaypointIndex]);
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
void RouteContinuous::addPoint(meter_t x, meter_t y, bool error)
{
    constexpr static uint8_t errorColour[3] = {0xFF, 0, 0};
    constexpr static uint8_t correctColour[3] = {0, 0xFF, 0};

    const float position[2] = {(float) x.value(), (float) y.value()};

    // Check we haven't run out of buffer space
    BOB_ASSERT(m_RouteNumPoints < m_RouteMaxPoints);

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
//----------------------------------------------------------------------------
void RouteContinuous::createGeometry()
{
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
}
}   // namespace AntWorld
}   // namespace BoBRobotics
