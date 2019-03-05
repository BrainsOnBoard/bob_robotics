// BoB robotics includes
#include "common/circstat.h"

TEST(CircStatTest, NormaliseAngle180) {
    ASSERT_UNIT_EQ(normaliseAngle180(360_deg), 0_deg);
    ASSERT_UNIT_EQ(normaliseAngle180(-360_deg), 0_deg);
    ASSERT_UNIT_EQ(normaliseAngle180(540_deg), 180_deg);
    ASSERT_UNIT_EQ(normaliseAngle180(-540_deg), 180_deg);
    ASSERT_UNIT_EQ(normaliseAngle180(180_deg), 180_deg);
    ASSERT_UNIT_EQ(normaliseAngle180(-180_deg), 180_deg);
    ASSERT_UNIT_EQ(normaliseAngle180(720_deg), 0_deg);
    ASSERT_UNIT_EQ(normaliseAngle180(-720_deg), 0_deg);
}

TEST(CircStatTest, CircularDistance) {
    ASSERT_UNIT_EQ(circularDistance(360_deg, 0_deg), 0_deg);
    ASSERT_UNIT_EQ(circularDistance(0_deg, 360_deg), 0_deg);
    ASSERT_UNIT_EQ(circularDistance(180_deg, -180_deg), 0_deg);
    ASSERT_UNIT_EQ(circularDistance(-180_deg, 180_deg), 0_deg);
    ASSERT_UNIT_EQ(circularDistance(180_deg, -170_deg), -10_deg);
    ASSERT_UNIT_EQ(circularDistance(-180_deg, 170_deg), 10_deg);
}
