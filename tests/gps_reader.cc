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
    std::ifstream ifs{ (Path::getProgramDirectory() / "sample_nmea.txt").str() };
    ASSERT_TRUE(ifs.good());

    GPS::NMEAParser nmea;
    std::string line;
    std::tm time;
    while (std::getline(ifs, line)) {
        line.pop_back(); // Remove trailing \r
        GPS::GPSReader::parseLine(nmea, line, time);
    }
}
