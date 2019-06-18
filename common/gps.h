//GPS


#include "nmea_parser.h"
#include "serial_reader.h"
#include "../third_party/units.h"
#include "map_coordinate.h"

namespace BoBRobotics 
{
namespace GPS 
{

class Gps {

    private:
    
    SerialReader sreader;
    const char *m_device_path;
    
    public:

    Gps(const char *device_path) : m_device_path(device_path) {

        try {
            sreader.connect(device_path);
            if (!sreader.isConnected()) throw "device not found";
            m_device_path = device_path;
        } catch(const char* error) {
            std::cout << "[Gps:Gps()]: " << error <<  std::endl;
            throw error;
        }
    }

    GPSData getGPSData() {
        GPSData data;
        try {
            data = NMEAParser::parseNMEA(sreader.readData()); 
            return data;
        } catch(const char *s) {
            std::cout << "[Gps:getGPSPosition]: " << s <<  std::endl;
        }
        return data;
    }
};
} // GPS
} // BoBRobotics