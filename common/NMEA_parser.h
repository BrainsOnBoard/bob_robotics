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
    
    using meter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using arcminute_t = units::angle::arcminute_t;

    private:

    std::vector<std::string> parseNMEAstringGGA(const std::string textToParse) {
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

    bool parseText(const std::string toParse, degree_t latitude) {
        using namespace std;
        vector<string> elements= parseNMEAstringGGA(toParse);
        string timeUTC = elements[1];
        

        latitude = degree_t(std::stod(elements[2].substr(0,2)));
        arcminute_t m_latMinute = arcminute_t(std::stod(elements[2].substr(2,8)));     
        string latDirection = elements[3];
        degree_t longitude = degree_t(std::stod(elements[4].substr(0,3)));
        arcminute_t longMinute = arcminute_t(std::stod(elements[4].substr(3,9)));
        string longDirection = elements[5];
        int gpsQualityIndicator = stoi(elements[6]);
        int numSats = stoi(elements[7]);
        double horizontalDilution = stod(elements[8]);
        meter_t altitude = meter_t(stod(elements[9]));
        
    }


};