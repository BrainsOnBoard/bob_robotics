#include "common.h"

// BoB robotics includes
#include "gps/nmea_parser.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <sstream>

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

/*
 * Check that a broken NMEA sentence triggers an NMEAError. Note that we append
 * the correct checksum so that the parsing doesn't fail on those grounds.
 */
#define TEST_BROKEN_NMEA(name, line)                                                       \
    TEST(NMEAParser, name)                                                                 \
    {                                                                                      \
        NMEAParser nmea;                                                                   \
        GPSData data{};                                                                    \
                                                                                           \
        std::stringstream ss;                                                              \
        const auto checksum = NMEAParser::computeChecksum(line, strlen(line));             \
        ss << line << "*"                                                                  \
           << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << checksum; \
                                                                                           \
        EXPECT_THROW(nmea.parseCoordinates(ss.str(), data), NMEAError);                    \
    }

TEST(NMEAParser, BadChecksum)
{
    NMEAParser nmea;
    GPSData data{};
    EXPECT_THROW(nmea.parseCoordinates("$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*46",
                                       data),
                 NMEAError);
}

TEST(NMEAParser, InvalidChecksum)
{
    NMEAParser nmea;
    GPSData data{};
    EXPECT_THROW(nmea.parseCoordinates("$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*xx",
                                       data),
                 NMEAError);
}

// We don't use the last few fields anyway, but let's test for if one of the ones we want is missing
TEST_BROKEN_NMEA(TooFewFields, "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12")
TEST_BROKEN_NMEA(BadLatitude, "$GNGGA,A001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M")
TEST_BROKEN_NMEA(BadNumSatellites, "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,A12,0.98,1113.0,M,-21.3,M")
TEST_BROKEN_NMEA(BadLatitudeDir, "$GNGGA,001043.00,4404.14036,E,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M")
TEST_BROKEN_NMEA(BadLongitudeDir, "$GNGGA,001043.00,4404.14036,N,12118.85961,S,1,12,0.98,1113.0,M,-21.3,M")
