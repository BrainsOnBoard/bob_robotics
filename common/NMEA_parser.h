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

#include<string>
#include<vector>
#include "../third_party/units.h"

class NMEA_parser {
    
    using meter_t = units::length::meter_t;
    using degree_t = units::angle::degree_t;
    using arcminute_t = units::angle::arcminute_t;
    

    private:
  

    static std::vector<std::string> parseNMEAstringGGA(const std::string textToParse) {
        const std::string delimiter = "$";      // sentences separated by $
        const std::string w_delimiter = ",";    // elements separated by ,
        std::string s = textToParse;

        size_t pos = 0;
        std::string token;
        std::vector<std::string> tokens,words;

        // separating the lines 
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            tokens.push_back(token);
            s.erase(0, pos + delimiter.length());
        }

        // separating the words
        for (auto &f : tokens) {         
            std::string word;
            pos = 0;   
            while ((pos = f.find(w_delimiter)) != std::string::npos) {
                word = f.substr(0,pos);
                words.push_back(word);
                f.erase(0, pos + w_delimiter.length());         
            }

            // if we find gngga, we stop 
            if (!words.empty() && words.at(0) == "GNGGA") {          
                break;
            } else {
                words.clear();
            }       
        }
        return words;
    }

    
    public:

    static bool parseTextGPS(const std::string &toParse,             // text to parse
                             degree_t          &latitude,           // latitude
                             arcminute_t       &latitudeMinutes,    // minute part of latitude
                             std::string       &latDirection,       // latitude direction North or South (N | S)
                             degree_t          &longitude,          // longitude
                             arcminute_t       &longitudeMinutes,   // minute part of longitude
                             std::string       &longDirection,      // longitude direction East or West (E | W)
                             meter_t           &altitude            // altitude in meters
                             ) {

        using namespace std;
        vector<string> elements= parseNMEAstringGGA(toParse); // parse string 
        latitude = degree_t(std::stod(elements[2].substr(0,2)));
        latitudeMinutes = arcminute_t(std::stod(elements[2].substr(2,8)));     
        latDirection = elements[3];
        longitude = degree_t(std::stod(elements[4].substr(0,3)));
        longitudeMinutes = arcminute_t(std::stod(elements[4].substr(3,9)));
        longDirection = elements[5];
        altitude = meter_t(stod(elements[9]));
        return true;
    }

    static bool parseTextOther(const std::string &toParse,          // text to parse
                               std::string &timeUTC,                // time UTC
                               int         &gpsQualityIndicator,    // 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
                               int         &numberOfSatelites,      // number of satelites
                               double      &horizontalDilution      // horizontal dilution, lower is better
                              ) {

        using namespace std;
        vector<string> elements= parseNMEAstringGGA(toParse); // parse string 
        timeUTC = elements[1]; // need to store and return somehow
        gpsQualityIndicator = stoi(elements[6]);
        numberOfSatelites = stoi(elements[7]);
        horizontalDilution = stod(elements[8]);
    }


};