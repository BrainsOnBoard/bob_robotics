// BoB robotics includes
#include "common/macros.h"
#include "gps/nmea_parser.h"

namespace BoBRobotics {
namespace GPS {

void
parseTime(const std::string &field, std::tm &time, int &milliseconds)
{
    if (field.empty()) {
        throw NMEAError("Empty string when reading time");
    }
    if (field.length() < 9) {
        throw NMEAError("Time information parse error");
    }

    time.tm_hour = stoi(field.substr(0, 2));
    time.tm_min = stoi(field.substr(2, 2));
    time.tm_sec = stoi(field.substr(4, 2));
    milliseconds = stoi(field.substr(7, 2));
}

bool
NMEAParser::parseCoordinates(const std::string &line, GPSData &data)
{
    using meter_t = units::length::meter_t;
    using degree_t = units::angle::degree_t;
    using arcminute_t = units::angle::arcminute_t;

    if (!parse(line, "$GNGGA", 9)) {
        return false;
    }

    try {
        parseTime(m_Fields[0], data.time, data.milliseconds); // UTC time

        data.coordinate.lat = degree_t(std::stod(m_Fields[1].substr(0, 2))) +
                              arcminute_t(std::stod(m_Fields[1].substr(2, 8)));
        if (m_Fields[2][0] == 'W') {
            data.coordinate.lat = -data.coordinate.lat;
        }
        data.coordinate.lon = degree_t(std::stod(m_Fields[3].substr(0, 3))) +
                              arcminute_t(std::stod(m_Fields[3].substr(3, 9)));
        if (m_Fields[4][0] == 'S') {
            data.coordinate.lon = -data.coordinate.lon;
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
