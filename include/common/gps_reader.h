#pragma once

// BoB robotics includes
#include "nmea.h"
#include "serial_interface.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace GPS {

class GPSReader
{
public:
    GPSReader(const char *devicePath = SerialInterface::DefaultLinuxDevicePath);

    /*!
     * \brief Attempt to read GPS data from the serial device.
     *
     * Returns true if data was successfully read and false if no new data was
     * available. Throws an exception if the data is malformed.
     */
    bool read(GPSData &data);

    //! Set the underlying serial device to (non)blocking mode for reading
    void setBlocking(bool);

private:
    SerialInterface m_Serial;
    std::string m_Line;
};
} // GPS
} // BoBRobotics
