// BoB robotics includes
#include "common/macros.h"
#include "gps/nmea_parser.h"

namespace BoBRobotics {
namespace GPS {

void
parseTimeField(const std::string &field, std::tm &time)
{
    time.tm_hour = stoi(field.substr(0, 2));
    time.tm_min = stoi(field.substr(2, 2));
    time.tm_sec = stoi(field.substr(4, 2));

    // Indicates that information about DST is not available
    time.tm_isdst = -1;
}

bool
NMEAParser::parseDateTime(const std::string &line, std::tm &time)
{
    // If this line isn't a valid time and date message, return false
    if (!parse(line, "$GNZDA", 6)) {
        return false;
    }

    try {
        parseTimeField(m_Fields[0], time);
        time.tm_mday = stoi(m_Fields[1]);
        time.tm_mon = stoi(m_Fields[2]) - 1;
        time.tm_year = stoi(m_Fields[3]) - 1900;

        // This field is available in GNU/BSD but is not strict ISO C
        int hoursOff = m_Fields[4].empty() ? 0 : stoi(m_Fields[4]);
        int minutesOff = m_Fields[5].size() <= 3 ? 0 : stoi(m_Fields[5].substr(0, 2));
        time.tm_gmtoff = 3600 * hoursOff + 60 * minutesOff;
    } catch (std::invalid_argument &e) {
        throw NMEAError(e.what());
    } catch (std::out_of_range &e) {
        throw NMEAError(e.what());
    }

    return true;
}

bool
NMEAParser::parseCoordinates(const std::string &line, GPSData &data)
{
    using meter_t = units::length::meter_t;
    using degree_t = units::angle::degree_t;
    using arcminute_t = units::angle::arcminute_t;

    // If this line isn't a valid coordinate message, return false
    if (!parse(line, "$GNGGA", 9)) {
        return false;
    }

    try {
        parseTimeField(m_Fields[0], data.time); // UTC time
        data.milliseconds = stoi(m_Fields[0].substr(7, 2));

        data.coordinate.lat = degree_t(std::stod(m_Fields[1].substr(0, 2))) +
                              arcminute_t(std::stod(m_Fields[1].substr(2, 8)));
        switch (m_Fields[2][0]) {
            case 'N':
                break;
            case 'S':
                data.coordinate.lat = -data.coordinate.lat;
                break;
            default:
                throw NMEAError{ "Bad latitude direction" };
        }

        data.coordinate.lon = degree_t(std::stod(m_Fields[3].substr(0, 3))) +
                              arcminute_t(std::stod(m_Fields[3].substr(3, 9)));
        switch (m_Fields[4][0]) {
            case 'E':
                data.coordinate.lon = -data.coordinate.lon;
                break;
            case 'W':
                break;
            default:
                throw NMEAError{ "Bad longitude direction" };
        }

        data.gpsQuality = static_cast<GPSQuality>(std::stoi(m_Fields[5]));
        data.numberOfSatellites = std::stoi(m_Fields[6]);
        data.horizontalDilution = meter_t{ std::stod(m_Fields[7]) };
        data.coordinate.height = meter_t{ std::stod(m_Fields[8]) };
    } catch (std::invalid_argument &e) {
        throw NMEAError(e.what());
    } catch (std::out_of_range &e) {
        throw NMEAError(e.what());
    }

    return true;
}

unsigned int
NMEAParser::computeChecksum(const char *str, size_t len)
{
    unsigned int checksum = 0;

    // Assume that the first char is $, which we skip
    for (size_t i = 1; i < len; i++) {
        checksum ^= str[i];
    }

    return checksum;
}

bool
NMEAParser::parse(const std::string &line, const std::string &sentenceId,
                  size_t numFields)
{
    BOB_ASSERT(numFields <= m_Fields.size());

    std::string field;
    std::istringstream lineStream{ line };

    // The first field indicates what kind of message we have
    if (!getline(lineStream, field, ',')) {
        return false;
    }
    if (field.empty() || field[0] != '$') {
        throw NMEAError{ "Sentence doesn't begin with $" };
    }
    if (field != sentenceId) {
        return false;
    }

    if (line.size() < 3 || line[line.size() - 3] != '*') {
        throw NMEAError{ "Missing checksum" };
    }

    // Check that the checksum matches
    char *p;
    unsigned int reportedChecksum = std::strtoul(&line[line.size() - 2], &p, 16);
    if (*p != 0) {
        throw NMEAError{ "Could not parse checksum" };
    }

    unsigned int actualChecksum = computeChecksum(line.c_str(), line.size() - 3);
    if (reportedChecksum != actualChecksum) {
        std::stringstream ss;
        ss << "Bad checksum (expected: " << std::hex << reportedChecksum
           << ", got: " << std::hex << actualChecksum << ")";
        throw NMEAError{ ss.str() };
    }

    // Extract the remaining fields
    size_t count = 0;
    for (; getline(lineStream, field, ',') && count < numFields; count++) {
        m_Fields[count] = std::move(field);
    }
    if (count != numFields) {
        throw NMEAError{ "Not enough fields in NMEA string" };
    }

    return true;
}
} // GPS
} // BoBRobotics
