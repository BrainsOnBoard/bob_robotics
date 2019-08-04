// Standard includes
#include <stdexcept>
// BoB includes
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
    bool isConnected;
    
    public:

    Gps() {}

    Gps(const char *device_path) : m_device_path(device_path), isConnected(false) {
        connect(device_path);
    }

    void connect(const char *device_path) {     
        sreader.connect(device_path);
        if (!sreader.isConnected()) throw GPSError("device not found");
        m_device_path = device_path;
        isConnected = true;
       
    }

    GPSData getGPSData() {
        GPSData data;
        if (!isConnected) throw GPSError("Not connected to the device");
        data = NMEAParser::parseNMEA(sreader.readData()); 
        return data;
    }
};
} // GPS
} // BoBRobotics