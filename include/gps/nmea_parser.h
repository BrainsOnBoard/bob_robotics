#pragma once

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

// BoB robotics includes
#include "common/map_coordinate.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <array>
#include <iomanip>
#include <sstream>
#include <string>

namespace BoBRobotics {
namespace GPS {

class NMEAError
  : public std::runtime_error
{
public:
    NMEAError(const std::string &msg)
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
            throw NMEAError("Empty string when reading time");
        if (timeString.length() < 9)
            throw NMEAError("Time information parse error");
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
    MapCoordinate::GPSCoordinate coordinate;   // Latitude and longitude coordinate
    units::length::meter_t altitude;           // Altitude above ellipsoid
    int numberOfSatellites;                    // Currently observed number of satelites
    units::length::meter_t horizontalDilution; // horizontal dilution - lower value is better
    GPSQuality gpsQuality;                     // GPS quality indicator
    TimeStamp time;                            // time of measurement
};

class NMEAParser
{
public:
    bool parseCoordinates(const std::string &line, GPSData &data);

private:
    std::array<std::string, 9> m_Fields; // For storing fields of NMEA sentences

    bool parse(const std::string &line, const std::string &sentenceID,
               size_t numFields);
};

} // GPS
} // BoBRobotics
