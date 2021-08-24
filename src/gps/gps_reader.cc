#ifndef WIN32

// BoB includes
#include "common/stopwatch.h"
#include "common/string.h"
#include "gps/gps_reader.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <stdexcept>

using namespace std::literals;

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

bool
GPSReader::readLine()
{
    do {
        // If there's no data available then we're done
        if (!m_Serial.read(m_Line)) {
            return false;
        }

        // If we got an empty line, try again
    } while (m_Line.empty());

    // Trim trailing CRLF
    strTrimRight(m_Line);

    return true;
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
    Stopwatch sw;
    sw.start();
    while (sw.elapsed() < 3s) {
        try {
            const auto data = read().value();
            if (data.gpsQuality != GPSQuality::INVALID) {
                LOGI << "Valid GPS fix found (" << data.coordinate.lat.value() << "°, "
                    << data.coordinate.lon.value() << "°)";
                return;
            }
        } catch (NMEAError &e) {
            LOGW << "NMEA parsing error: " << e.what() << ": " << m_Line;
        }
    }

    throw std::runtime_error{ "There is no valid gps measurement, please try waiting for the survey in to finish and restart the program" };
}

void
GPSReader::waitForCurrentTime()
{
    // We may already have read a GNZDA message
    if (m_CurrentTime.tm_year != 0) {
        return;
    }

    // Try to read the current time + date for 3s
    Stopwatch sw;
    sw.start();
    while (sw.elapsed() < 3s) {
        // This method will block
        readLine();

        try {
            // If it's a GNZDA message, we want to extract the time + date info
            const auto time = m_Parser.parseDateTime(m_Line);
            if (time) {
                m_CurrentTime = time.value();
                return;
            }
        } catch (NMEAError &e) {
            LOGW << "NMEA parsing error: " << e.what() << ": " << m_Line;
        }
    }

    throw std::runtime_error{ "Timed out waiting for GPS signal" };
}

void
GPSReader::setBlocking(bool blocking)
{
    auto tty = m_Serial.getAttributes();
    tty.c_cc[VMIN] = blocking ? 1 : 0;
    m_Serial.setAttributes(tty);
}

std::experimental::optional<GPSData>
GPSReader::parseLine(NMEAParser &parser,
                     const std::string &line,
                     std::tm &currentTime)
{
    if (auto optData = parser.parseCoordinates(line)) {
        /*
         * We have GPS coordinates! Copy date and timezone info from
         * cached GNZDA data.
         */
        auto &data = optData.value();
        data.time.tm_mday = currentTime.tm_mday;
        data.time.tm_mon = currentTime.tm_mon;
        data.time.tm_year = currentTime.tm_year;
        data.time.tm_gmtoff = currentTime.tm_gmtoff;

        return optData;
    }

    // If it's a GNZDA message, we want to extract the time + date info
    if (const auto optTime = parser.parseDateTime(line)) {
        currentTime = optTime.value();
    }

    return std::experimental::nullopt;
}

std::experimental::optional<GPSData>
GPSReader::read()
{
    /*
     * If socket is non-blocking, then this will return false if there is no new
     * data.
     */
    while (readLine()) {
        try {
            if (auto optData = parseLine(m_Parser, m_Line, m_CurrentTime)) {
                return optData;
            }
        } catch (NMEAError &e) {
            LOGW << "NMEA parsing error: " << e.what() << ": " << m_Line;

            try {
                std::rethrow_if_nested(e);
            } catch(const std::exception& e) {
                LOGW << "NMEA parsing error internal: " <<  e.what();
            }
        }
    }

    return std::experimental::nullopt;
}

} // GPS
} // BoBRobotics

#endif
