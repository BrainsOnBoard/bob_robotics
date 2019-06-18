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
#include<string>
#include<vector>
#include <sstream>
#include "../third_party/units.h"
#include "map_coordinate.h"

namespace BoBRobotics
{
namespace GPS 
{

enum class GPSQuality
{
    INVALID,  
    GPSFIX,  
    DGPSFIX,
    PPSFIX,
    RTK,
    FRTK,
};

struct TimeStamp
{
    units::time::hour_t hour;
    units::time::minute_t minute;
    units::time::second_t second;
    units::time::millisecond_t millisecond;
    TimeStamp(){}
    TimeStamp (const int &h, const int &m, const int &s, const int &ms)
    : hour(h), minute(m), second(s), millisecond(ms) {}
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
        try {
            if (textToParse == "") throw "Empty string to parse";

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
                throw "[NMEAParser:parseNMEAstring()] : cannot find NMEA id";
            }
            auto f = sentences[sentenceNumber];
            // separating the sentence to words 
            istringstream splitWord(f);
            for (string word; getline(splitWord, word, w_delimiter); words.push_back(word));

        } catch(const char *error) {
            throw error;
        } catch(...) {}

        return words;
    }

    static GPSQuality getGpsQuality(const int &qualityId) {     
        GPSQuality quality;  
        switch(qualityId) {
            case 0 : quality = GPSQuality::INVALID; break;
            case 1 : quality = GPSQuality::GPSFIX; break;
            case 2 : quality = GPSQuality::DGPSFIX; break;
            case 3 : quality = GPSQuality::PPSFIX; break;
            case 4 : quality = GPSQuality::RTK; break;
            case 5 : quality = GPSQuality::FRTK; break;
        }
        return quality;
    }

    // parsing the string time to numbers
    static TimeStamp parseStringTime(std::string timeString) {
        try {
            if (timeString== "") throw "Empty string when reading time";
            int hour = stoi(timeString.substr(0,2));
            int min  = stoi(timeString.substr(2,2));
            int sec  = stoi(timeString.substr(4,2));
            int msc  = stoi(timeString.substr(7,2));
            TimeStamp time(hour, min, sec, msc);    
            return time;
        } catch(const char *s) {
            throw s;
        }  catch(...) { }
        
        TimeStamp time(0,0,0,0);
        return time;
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
            if (toParse == "") throw "Emtpy serial output";
            vector<string> elements= parseNMEAstring(toParse, "GNGGA"); // parse string 
            string timeString = elements[1];
            TimeStamp time = parseStringTime(timeString);
            latitude = degree_t(stod(elements[2].substr(0,2)));
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
            qualityOfGps = getGpsQuality(gpsQualityIndicator);

            // adding up the latitude and longitude parts
            latitude = latitude + latitudeMinutes;
            longitude = longitude + longitudeMinutes;

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
        } catch(const char *error) {
            throw error;
        } catch(...) { }
        return data;
    } 
};
} // GPS
} // BoBRobotics
