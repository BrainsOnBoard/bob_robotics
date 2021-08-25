#include "common.h"

// BoB robotics includes
#include "gps/nmea_parser.h"

// Standard C++ includes
#include <sstream>

using namespace BoBRobotics::GPS;
using namespace units::angle;
using namespace units::length;


TEST(NMEAParser, SampleGNGGASentence)
{
    NMEAParser nmea;

    const auto sentence = "$GNGGA,001043.00,4404.14036,N,12118.85961,W,"
                          "1,12,0.98,1113.0,M,-21.3,M*47";
    const auto data = nmea.parseCoordinates(sentence).value();

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

    const auto time = nmea.parseDateTime("$GNZDA,143042.00,25,08,2005,,*70").value();

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

TEST(NMEAParser, BadChecksum)
{
    NMEAParser nmea;
    EXPECT_THROW(nmea.parseCoordinates("$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*46"),
                 NMEAError);
}

TEST(NMEAParser, InvalidChecksum)
{
    NMEAParser nmea;
    EXPECT_THROW(nmea.parseCoordinates("$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*xx"),
                 NMEAError);
}

/*
 * Check that a broken NMEA sentence triggers an NMEAError. Note that we append
 * the correct checksum so that the parsing doesn't fail on those grounds.
 */
void
testBrokenNMEA(const char *sentence)
{
    NMEAParser nmea;

    std::stringstream ss;
    const auto checksum = NMEAParser::computeChecksum(sentence, strlen(sentence));
    ss << sentence << "*"
       << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << checksum;

    EXPECT_THROW(nmea.parseCoordinates(ss.str()), NMEAError);
}

// We don't use the last few fields anyway, but let's test for if one of the ones we want is missing
TEST(NMEAParser, TooFewFields)
{
    testBrokenNMEA("$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12");
}

TEST(NMEAParser, BadLatitude)
{
    testBrokenNMEA("$GNGGA,A001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M");
}

TEST(NMEAParser, BadNumSatellites)
{
    testBrokenNMEA("$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,A12,0.98,1113.0,M,-21.3,M");
}

TEST(NMEAParser, BadLatitudeDir)
{
    testBrokenNMEA("$GNGGA,001043.00,4404.14036,E,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M");
}

TEST(NMEAParser, BadLongitudeDir)
{
    testBrokenNMEA("$GNGGA,001043.00,4404.14036,N,12118.85961,S,1,12,0.98,1113.0,M,-21.3,M");
}
