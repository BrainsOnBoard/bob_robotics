//GPS


#include "NMEA_parser.h"
#include "serial_reader.h"
#include "../third_party/units.h"




class GPS {
    using namespace units::angle;
    using namespace units::length;

    private:
    NMEA_parser m_parser;
    Serial_reader m_reader;
    int m_currentNumberOfSats;
    meter_t m_altitude;
    




    public:
    std::array<units::angle::degree_t, 2> getGPSPosition() {
        
    }

    units::length::millimeter_t getAltitude() {

    }

}