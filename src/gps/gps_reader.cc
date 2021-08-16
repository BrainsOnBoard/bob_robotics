// BoB includes
#include "common/stopwatch.h"
#include "gps/gps_reader.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
namespace GPS {

constexpr const char *GPSReader::DefaultLinuxDevicePath;

GPSReader::GPSReader(const char *devicePath)
  : m_Serial{ devicePath }
{
    // Set baudrate etc.
    setSerialAttributes();

    // Check that we actually have valid readings
    waitForValidReading();

    // These messages should be emitted roughly every ~1s by the ublox sensor
    waitForCurrentTime();
}

const std::tm &
GPSReader::getCurrentDateTime() const
{
    return m_CurrentTime;
}

void
GPSReader::setSerialAttributes()
{
    termios tty{};

    /*
     * B9600   : set baudrate to 9600
     * CS8     : 8n1 (8bit,no parity,1 stopbit)
     * CLOCAL  : local connection, no modem contol
     * CREAD   : enable receiving characters
     */
    tty.c_cflag = B9600 | CRTSCTS | CS8 | CLOCAL | CREAD;

    // ICANON  : enable canonical input
    tty.c_lflag = ICANON;

    // IGNPAR  : ignore bytes with parity errors
    tty.c_iflag = IGNPAR;

    /*  Raw output  */
    tty.c_oflag = (OPOST | ONLCR);

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1; // set to blocking by default
    tty.c_cc[VTIME] = 5; // set timeout to 0.5 secs

    m_Serial.setAttributes(tty);
}

void
GPSReader::waitForValidReading()
{
    // if GPS location is invalid, keep trying to get a valid one
    // if failed x times we exit
    int numTrials = 3;
    GPSData data{};
    while (numTrials--) {
        read(data);
        if (data.gpsQuality != GPSQuality::INVALID) {
            LOGI << "Valid GPS fix found (" << data.coordinate.lat.value() << "°, "
                 << data.coordinate.lon.value() << "°)";
            break;
        }
    }

    if (numTrials == 0) {
        throw std::runtime_error{ "There is no valid gps measurement, please try waiting for the survey in to finish and restart the program" };
    }
}

void
GPSReader::waitForCurrentTime()
{
    using namespace std::literals;

    // We may already have read a GNZDA message
    if (m_CurrentTime.tm_year != 0) {
        return;
    }

    // Try to read the current time + date for 5s
    Stopwatch sw;
    sw.start();
    while (sw.elapsed() < 5s) {
        // This method will block
        m_Serial.read(m_Line);

        try {
            // If it's a GNZDA message, we want to extract the time + date info
            if (m_Parser.parseDateTime(m_Line, m_CurrentTime)) {
                return;
            }
        } catch (NMEAError &e) {
            LOGW << "NMEA parsing error: " << e.what() << ": " << m_Line;
        }
    }
}

void
GPSReader::setBlocking(bool blocking)
{
    auto tty = m_Serial.getAttributes();
    tty.c_cc[VMIN] = blocking ? 1 : 0;
    m_Serial.setAttributes(tty);
}

bool
GPSReader::read(GPSData &data)
{
    while (true) {
        // If in non-blocking mode and there's no data, return
        if (!m_Serial.read(m_Line)) {
            return false;
        }

        try {
            if (m_Parser.parseCoordinates(m_Line, data)) {
                // Signal that we have successfully read valid GPS data
                return true;
            }

            // Copy date and timezone info from cached GNZDA data
            data.time.tm_mday = m_CurrentTime.tm_mday;
            data.time.tm_mon = m_CurrentTime.tm_mon;
            data.time.tm_year = m_CurrentTime.tm_year;
            data.time.tm_gmtoff = m_CurrentTime.tm_gmtoff;

            // If it's a GNZDA message, we want to extract the time + date info
            m_Parser.parseDateTime(m_Line, m_CurrentTime);
        } catch (NMEAError &e) {
            LOGW << "NMEA parsing error: " << e.what() << ": " << m_Line;
        }
    }
}

} // GPS
} // BoBRobotics
