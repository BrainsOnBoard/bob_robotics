#include "common.h"

// BoB robotics includes
#include "common/path.h"
#include "gps/gps_reader.h"

// Standard C++ includes
#include <fstream>
#include <string>

using namespace BoBRobotics;


TEST(GPSReader, ParseSampleData)
{
    std::ifstream ifs{ (getTestsPath() / "sample_nmea.txt").str() };
    ASSERT_TRUE(ifs.good());

    GPS::NMEAParser nmea;
    std::string line;
    std::tm time;
    while (std::getline(ifs, line)) {
        if (!line.empty()) {
            line.pop_back(); // Remove trailing \r
        }

        GPS::GPSReader::tryParseLine(nmea, line, time);
    }
}
