//GPS


#include "NMEA_parser.h"
#include "serial_reader.h"
#include "../third_party/units.h"
#include "map_coordinate.h"




class Gps {
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;
  
    private:
    
    int m_currentNumberOfSats;
    int m_gpsQuality;
    double m_horizontalDilution;
    meter_t m_altitude;
    degree_t m_longitude;
    degree_t m_latitude;

    void getGPSPosition() {
        using namespace units::angle;
        using namespace units::literals;

        arcminute_t latmin;
        arcminute_t longmin;
        degree_t lat;
        degree_t longitude;
        std::string latDir;
        std::string longDir;

        // reading and parsing serial data
        std::string serialString =  Serial_reader::readSerialUSB();
        NMEA_parser::parseTextGPS(serialString, lat, latmin, latDir, longitude, longmin, longDir, m_altitude);
        m_latitude = lat + latmin;
        m_longitude = longitude + longmin;

        // West and South has negative angles
        if (longDir == "W") m_longitude = 0_deg - m_longitude;
        if (latDir == "S")  m_latitude  = 0_deg - m_latitude;

    }

    void getGpsDescription() {
        std::string serialString =  Serial_reader::readSerialUSB();
        int quality, numsat;
        double horizontalDilution;
        NMEA_parser::parseTextForMiscData(serialString, quality, numsat, horizontalDilution);
        m_gpsQuality = quality;
        m_currentNumberOfSats = numsat;
        m_horizontalDilution = horizontalDilution;
    }


    public:

    bool getPosition(degree_t &lat, degree_t &lon, meter_t &altitude) {
        getGPSPosition(); 
        lat = m_latitude;
        lon = m_longitude;
        altitude = m_altitude;
        return true;
    }

    int getNumberOfSatelites() {
        getGpsDescription();
        return m_currentNumberOfSats;
    }

    int getGpsQuality() {
        getGpsDescription();
        return m_gpsQuality;
    }

    double getHorizontalDilution() {
        getGpsDescription();
        return m_horizontalDilution;
    }


   

};