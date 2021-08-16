#include "common.h"

// BoB robotics includes
#include "gps/nmea_parser.h"

// Standard C includes
#include <cstring>

using namespace BoBRobotics::GPS;
using namespace units::angle;
using namespace units::length;


TEST(NMEAParser, SampleGNGGASentence)
{
    NMEAParser nmea;
    GPSData data;

    const auto sentence = "$GNGGA,001043.00,4404.14036,N,12118.85961,W,"
                          "1,12,0.98,1113.0,M,-21.3,M*47";
    ASSERT_TRUE(nmea.parseCoordinates(sentence, data));

    // Check timestamp
    EXPECT_EQ(data.time.tm_hour, 0);
    EXPECT_EQ(data.time.tm_min, 10);
    EXPECT_EQ(data.time.tm_sec, 43);
    EXPECT_EQ(data.milliseconds, 0);

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

TEST(NMEAParser, SampleGNZDASentence)
{
    NMEAParser nmea;
    std::tm time;

    // Don't leave fields zero-initialised, for the sake of this test
    memset(&time, 0xff, sizeof(time));

    ASSERT_TRUE(nmea.parseDateTime("$GNZDA,143042.00,25,08,2005,,*70", time));

    // Time
    EXPECT_EQ(time.tm_hour, 14);
    EXPECT_EQ(time.tm_min, 30);
    EXPECT_EQ(time.tm_sec, 42);

    // Date
    EXPECT_EQ(time.tm_mday, 25);
    EXPECT_EQ(time.tm_mon, 7);    // NB: tm_mon is in range [0, 11]
    EXPECT_EQ(time.tm_year, 105); // NB: tm_year starts counting from 1900

    // Timezone stuff (here it's just UTC anyway)
    EXPECT_EQ(time.tm_gmtoff, 0);
    EXPECT_EQ(time.tm_isdst, -1);
}
