// BoB includes
#include "nmea.h"
#include "serial_interface.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace GPS {

class GPSReader
{
public:
    GPSReader(const char *devicePath);
    bool read(GPSData &data);
    void setBlocking(bool);

private:
    SerialInterface m_Serial;
    std::string m_Line;
};
} // GPS
} // BoBRobotics
