#include "common.h"

// BoB robotics includes
#include "gps/nmea_parser.h"

using namespace BoBRobotics::GPS;
using namespace units::angle;
using namespace units::length;

const auto sentence = "$GNGGA,001043.00,4404.14036,N,12118.85961,W,"
                      "1,12,0.98,1113.0,M,-21.3,M*47";

TEST(NMEAParser, SampleGNGGASentence)
{
    NMEAParser nmea;
    GPSData data;

    nmea.parseCoordinates(sentence, data);

    // Check timestamp
    EXPECT_EQ(data.time.hour, 0);
    EXPECT_EQ(data.time.minute, 10);
    EXPECT_EQ(data.time.second, 43);
    EXPECT_EQ(data.time.millisecond, 0);

    // Check coordinates
    BOB_EXPECT_UNIT_T_EQ(data.coordinate.lat, degree_t{ 44 } + arcminute_t{ 4.14036 });
    BOB_EXPECT_UNIT_T_EQ(data.coordinate.lon, degree_t{ 121 } + arcminute_t{ 18.85961 });
    BOB_EXPECT_UNIT_T_EQ(data.coordinate.height, meter_t{ 1113 });

    // Check self-reported accuracy
    BOB_EXPECT_UNIT_T_EQ(data.horizontalDilution, meter_t{ 0.98 });

    // Misc
    EXPECT_EQ(data.gpsQuality, GPSQuality::GPSFIX);
    EXPECT_EQ(data.numberOfSatellites, 12);
}
