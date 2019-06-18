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
// standard includes
#include<string>
#include<vector>
#include <sstream>

// BoB includes
#include "../third_party/units.h"
#include "map_coordinate.h"

namespace BoBRobotics
{
namespace GPS
{

class GPSError
  : public std::runtime_error
{
public:
    GPSError(const std::string &msg) : std::runtime_error(msg) { }
};

enum class GPSQuality
{
    INVALID = 0,
    GPSFIX  = 1,
    DGPSFIX = 2,
    PPSFIX  = 3,
    RTK     = 4,
    FRTK    = 5
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
};

struct GPSData {
    BoBRobotics::MapCoordinate::GPSCoordinate coordinate;  // Latitude and longitude coordinate
    units::length::meter_t altitude;                       // Altitude above ellipsoid
    units::velocity::meters_per_second_t velocity;         // velocity
    int numberOfSatelites;                                 // Currently observed number of satelites
    double horizontalDilution;                             // horizontal dilution - lower value is better
    GPSQuality gpsQuality;                                 // GPS quality indicator
    TimeStamp time;                                        // time of measurement
};

class NMEAParser {

    using second_t = units::time::second_t;
    using minute_t = units::time::minute_t;
    using hour_t = units::time::hour_t;
    using millisecond_t = units::time::millisecond_t;
    using meter_t = units::length::meter_t;
    using degree_t = units::angle::degree_t;
    using arcminute_t = units::angle::arcminute_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

    private:

    static std::vector<std::string> parseNMEAstring(const std::string &textToParse, const std::string &NMEA_sentence_id) {

        using namespace std;
        const char delimiter = '$';      // sentences separated by [$]
        const char w_delimiter = ',';    // elements separated by  [,]

        vector<string> sentences,words;
        istringstream split(textToParse);
        // separating the text to sentences

        if (textToParse.empty()) throw GPSError("Empty string to parse");

        for (string each; getline(split, each, delimiter); sentences.push_back(each));
        // find the sentence with the id we want
        int sentenceNumber= -1;
        for (vector<int>::size_type i = 0; i < sentences.size(); i++) {
            size_t found = sentences[i].find(NMEA_sentence_id);
            if (found!= string::npos) {
                sentenceNumber = i;
                break;
            }
        }
        if (sentenceNumber < 0) {
            throw GPSError("cannot find NMEA id");
        }
        auto f = sentences[sentenceNumber];
        // separating the sentence to words
        istringstream splitWord(f);
        for (string word; getline(splitWord, word, w_delimiter); words.push_back(word));

        return words;
    }

    public:

    static GPSData parseNMEA(const std::string  &toParse) {
        using namespace std;

        degree_t            latitude, longitude;
        arcminute_t         latitudeMinutes, longitudeMinutes;
        char                latDirection, longDirection;
        meter_t             altitude;
        meters_per_second_t velocity;
        double              horizontalDilution;
        int                 numberOfSatelites;
        int                 gpsQualityIndicator;
        GPSQuality          qualityOfGps;
        GPSData             data;

        try {
            if (toParse.empty()) throw GPSError("Emtpy serial output");
            vector<string> elements= parseNMEAstring(toParse, "GNGGA"); // parse string
            if (elements.size() < 10) throw GPSError("Wrong number of elements when parsing the string");
            string timeString = elements[1];
            TimeStamp time{ timeString };
            latitude = degree_t(stod(elements[2].substr(0, 2)));
            latitudeMinutes = arcminute_t(stod(elements[2].substr(2,8)));
            latDirection = elements[3][0];
            longitude = degree_t(stod(elements[4].substr(0,3)));
            longitudeMinutes = arcminute_t(stod(elements[4].substr(3,9)));
            longDirection = elements[5][0];
            gpsQualityIndicator = stoi(elements[6]);
            numberOfSatelites = stoi(elements[7]);
            horizontalDilution = stod(elements[8]);
            altitude = meter_t(stod(elements[9]));
            vector<string> elementsRMC= parseNMEAstring(toParse, "GNRMC"); // parse GNRMC for velocity
            velocity = units::velocity::knot_t(stod(elementsRMC[7])); // we change unit from knot to meters_per_second
            qualityOfGps = static_cast<GPSQuality>(gpsQualityIndicator);

            // adding up the latitude and longitude parts
            latitude += latitudeMinutes;
            longitude += longitudeMinutes;

            // West and South has negative angles
            if (longDirection == 'W') longitude = - longitude;
            if (latDirection == 'S')  latitude  = - latitude;

            BoBRobotics::MapCoordinate::GPSCoordinate coordinate;
            coordinate.lat = latitude;
            coordinate.lon = longitude;
            data.coordinate = coordinate;
            data.numberOfSatelites = numberOfSatelites;
            data.altitude = altitude;
            data.velocity = velocity;
            data.horizontalDilution = horizontalDilution;
            data.gpsQuality = qualityOfGps;
            data.time = time; // UTC time
        } catch(std::invalid_argument &e) {
            throw GPSError(e.what());
        } catch(std::out_of_range &e) {
            throw GPSError(e.what());
        }
        return data;
    }
};
} // GPS
} // BoBRobotics
