// NMEA message parser

/*
          1       2      3 4        5 6 7  8   9  10 11  12 13 14   15
|         |       |      | |        | | |  |   |   | |   | |   |    |
$--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh

1) Time (UTC)
2) Latitude
3) N or S (North or South)
4) Longitude
5) E or W (East or West)
6) GPS Quality Indicator,
0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
7) Number of satellites in view, 00 - 12
8) Horizontal Dilution of precision, lower is better
9) Antenna Altitude above/below mean-sea-level (geoid)
10) Units of antenna altitude, meters
11) Geoidal separation, the difference between the WGS-84 earth
ellipsoid and mean-sea-level (geoid), '-' means mean-sea-level below ellipsoid
12) Units of geoidal separation, meters
13) Age of differential GPS data, time in seconds since last SC104
type 1 or 9 update, null field when DGPS is not used
14) Differential reference station ID, 0000-1023
15) Checksum
*/

#pragma once
// BoB robotics includes
#include "common/macros.h"
#include "map_coordinate.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace GPS {

class GPSError
  : public std::runtime_error
{
public:
    GPSError(const std::string &msg)
      : std::runtime_error(msg)
    {}
};

enum class GPSQuality
{
    INVALID = 0,
    GPSFIX = 1,
    DGPSFIX = 2,
    PPSFIX = 3,
    RTK = 4,
    FRTK = 5
};

struct TimeStamp
{
    int hour{}, minute{}, second{}, millisecond{};

    TimeStamp() = default;

    TimeStamp(const std::string &timeString)
    {
        if (timeString.empty())
            throw GPSError("Empty string when reading time");
        if (timeString.length() < 9)
            throw GPSError("Time information parse error");
        hour = stoi(timeString.substr(0, 2));
        minute = stoi(timeString.substr(2, 2));
        second = stoi(timeString.substr(4, 2));
        millisecond = stoi(timeString.substr(7, 2));
    }

    std::string str() const
    {
        std::stringstream ss;
        ss << std::setw(2) << std::setfill('0') << hour << ":"
           << std::setw(2) << std::setfill('0') << minute << ":";
        if (second < 10) {
            ss << '0';
        }
        ss << static_cast<float>(second) + static_cast<float>(millisecond) / 1000.f;
        return ss.str();
    }
};

struct GPSData
{
    BoBRobotics::MapCoordinate::GPSCoordinate coordinate; // Latitude and longitude coordinate
    units::length::meter_t altitude;                      // Altitude above ellipsoid
    int numberOfSatellites;                               // Currently observed number of satelites
    double horizontalDilution;                            // horizontal dilution - lower value is better
    GPSQuality gpsQuality;                                // GPS quality indicator
    TimeStamp time;                                       // time of measurement
};

namespace NMEA {

template<size_t N>
bool
splitString(const std::string &line,
            std::array<std::string, N> &fields,
            const std::string &sentenceId)
{
    std::string field;
    std::istringstream lineStream{ line };

    // The first field indicates what kind of message we have
    BOB_ASSERT(getline(lineStream, field, ','));
    if (field != sentenceId) {
        return false;
    }

    // Extract the remaining fields
    size_t count = 0;
    for (; getline(lineStream, field, ',') && count < N; count++) {
        fields[count] = std::move(field);
    }
    if (count != N) {
        throw GPSError{ "Bad number of fields in NMEA string" };
    }

    return true;
}

inline bool
parse(const std::string &toParse, GPSData &data)
{
    using namespace std;
    using meter_t = units::length::meter_t;
    using degree_t = units::angle::degree_t;
    using arcminute_t = units::angle::arcminute_t;

    if (toParse.empty()) {
        return false;
    }

    degree_t latitude, longitude;
    arcminute_t latitudeMinutes, longitudeMinutes;
    char latDirection, longDirection;
    meter_t altitude;
    double horizontalDilution;
    int numberOfSatellites;
    int gpsQualityIndicator;
    GPSQuality qualityOfGps;

    // Let's reuse our data structure so we don't have to reallocate with every call
    static array<string, 9> elements;
    if (!splitString(toParse, elements, "$GNGGA")) {
        return false;
    }

    try {
        TimeStamp time{ elements[0] };
        latitude = degree_t(stod(elements[1].substr(0, 2)));
        latitudeMinutes = arcminute_t(stod(elements[1].substr(2, 8)));
        latDirection = elements[2][0];
        longitude = degree_t(stod(elements[3].substr(0, 3)));
        longitudeMinutes = arcminute_t(stod(elements[3].substr(3, 9)));
        longDirection = elements[4][0];
        gpsQualityIndicator = stoi(elements[5]);
        numberOfSatellites = stoi(elements[6]);
        horizontalDilution = stod(elements[7]);
        altitude = meter_t(stod(elements[8]));
        qualityOfGps = static_cast<GPSQuality>(gpsQualityIndicator);

        // adding up the latitude and longitude parts
        latitude += latitudeMinutes;
        longitude += longitudeMinutes;

        // West and South has negative angles
        if (longDirection == 'W')
            longitude = -longitude;
        if (latDirection == 'S')
            latitude = -latitude;

        MapCoordinate::GPSCoordinate coordinate;
        coordinate.lat = latitude;
        coordinate.lon = longitude;
        data.coordinate = coordinate;
        data.numberOfSatellites = numberOfSatellites;
        data.altitude = altitude;
        data.horizontalDilution = horizontalDilution;
        data.gpsQuality = qualityOfGps;
        data.time = time; // UTC time
    } catch (std::invalid_argument &e) {
        throw GPSError(e.what());
    } catch (std::out_of_range &e) {
        throw GPSError(e.what());
    }

    return true;
}
} // NMEA
} // GPS
} // BoBRobotics
