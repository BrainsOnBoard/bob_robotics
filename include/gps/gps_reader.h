#pragma once

// BoB robotics includes
#include "common/serial_interface.h"
#include "gps/nmea_parser.h"

// Third-party includes
#include "third_party/optional.hpp"

// Standard C++ includes
#include <string>

// Standard C includes
#include <ctime>

namespace BoBRobotics {
namespace GPS {

class GPSReader
{
    template<class T>
    using optional = std::experimental::optional<T>;

public:
    static constexpr const char *DefaultLinuxDevicePath = "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00";
    GPSReader(const char *devicePath = DefaultLinuxDevicePath);

    //! Note: This will probably be UTC
    const std::tm &getCurrentDateTime();

    /*!
     * \brief Attempt to read GPS data from the serial device.
     *
     * Returns true if data was successfully read and false if no new data was
     * available. Throws an exception if the data is malformed.
     */
    optional<GPSData> read();

    //! Set the underlying serial device to (non)blocking mode for reading
    void setBlocking(bool);

    static optional<GPSData> tryParseLine(NMEAParser &parser,
                                          const std::string &line,
                                          std::tm &currentTime);

private:
    SerialInterface m_Serial;
    NMEAParser m_Parser;
    std::string m_Line;
    std::tm m_CurrentTime{};

    bool readLine();
    void setSerialAttributes();
    void waitForValidReading();
    void waitForCurrentTime();
};
} // GPS
} // BoBRobotics
