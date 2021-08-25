#ifndef WIN32

// BoB includes
#include "common/stopwatch.h"
#include "common/string.h"
#include "gps/gps_reader.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <chrono>
#include <stdexcept>
#include <thread>

using namespace std::experimental;
using namespace std::literals;

namespace BoBRobotics {
namespace GPS {

constexpr const char *GPSReader::DefaultLinuxDevicePath;

void
printError(const NMEAError &e, const std::string &line)
{
    LOGW << "NMEA parsing error: " << e.what() << ": " << line;

    try {
        std::rethrow_if_nested(e);
    } catch (const std::exception &e) {
        LOGW << "NMEA parsing error internal: " << e.what();
    }
}

GPSReader::GPSReader(const char *devicePath)
  : m_Serial{ devicePath, /*blocking=*/false }
{
    // Set baudrate etc.
    setSerialAttributes();

    // Check that we actually have valid readings
    waitForValidReading();
}

const std::tm &
GPSReader::getCurrentDateTime()
{
    /*
     * If we've already read the current date + time once, then just return
     * that. Otherwise try reading more data.
     */
    if (m_CurrentTime.tm_year || !readLine()) {
        return m_CurrentTime;
    }

    // Might get the date + time, might not
    if (const auto time = m_Parser.parseDateTime(m_Line)) {
        m_CurrentTime = time.value();
    }

    /*
     * If all else fails, we will at least have time info because we know that
     * we've received at one valid GNGGA message (which contains the time).
     */
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
    tty.c_cc[VMIN] = 0;  // set to nonblocking
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
            const auto data = read();
            if (data && data->gpsQuality != GPSQuality::INVALID) {
                LOGI << "Valid GPS fix found (" << data->coordinate.lat.value() << "°, "
                     << data->coordinate.lon.value() << "°)";
                return;
            }
        } catch (NMEAError &e) {
            printError(e, m_Line);
        }

        std::this_thread::sleep_for(50ms);
    }

    throw std::runtime_error{ "There is no valid gps measurement, please try waiting for the survey in to finish and restart the program" };
}

optional<GPSData>
GPSReader::tryParseLine(NMEAParser &parser,
                        const std::string &line,
                        std::tm &currentTime)
{
    if (auto data = parser.parseCoordinates(line)) {
        // We have GPS coordinates!
        auto &time = data->time;

        /*
         * Try to copy date and timezone info from cached GNZDA data. Might not
         * be present.
         */
        time.tm_mday = currentTime.tm_mday;
        time.tm_mon = currentTime.tm_mon;
        time.tm_year = currentTime.tm_year;
        time.tm_gmtoff = currentTime.tm_gmtoff;

        /*
         * Update time portion of struct, so even if it doesn't have a date, at
         * least it will have a time.
         */
        currentTime.tm_hour = time.tm_hour;
        currentTime.tm_min = time.tm_min;
        currentTime.tm_sec = time.tm_sec;

        return data;
    }

    /*
     * If it's a GNZDA message, we want to extract the time + date info. We only
     * update currentTime if we get a valid reading so we don't clobber existing
     * data.
     */
    if (const auto time = parser.parseDateTime(line)) {
        currentTime = time.value();
    }

    return nullopt;
}

optional<GPSData>
GPSReader::read()
{
    optional<GPSData> data;

    /*
     * readLine() returns false if there is no new serial data.
     *
     * We keep trying to read data until there is no more available, then return
     * the last retrieved GPSData.
     */
    while (readLine()) {
        try {
            if (auto newData = tryParseLine(m_Parser, m_Line, m_CurrentTime)) {
                data = newData;
            }
        } catch (NMEAError &e) {
            printError(e, m_Line);
        }
    }

    return data;
}

} // GPS
} // BoBRobotics

#endif
