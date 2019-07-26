#include "common.h"

// BoB robotics includes
#include "common/circstat.h"

TEST(CircStatTest, NormaliseAngle180) {
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(360_deg), 0_deg);
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(-360_deg), 0_deg);
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(540_deg), 180_deg);
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(-540_deg), 180_deg);
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(180_deg), 180_deg);
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(-180_deg), 180_deg);
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(720_deg), 0_deg);
    BOB_EXPECT_UNIT_T_EQ(normaliseAngle180(-720_deg), 0_deg);
}

TEST(CircStatTest, CircularDistance) {
    BOB_EXPECT_UNIT_T_EQ(circularDistance(360_deg, 0_deg), 0_deg);
    BOB_EXPECT_UNIT_T_EQ(circularDistance(0_deg, 360_deg), 0_deg);
    BOB_EXPECT_UNIT_T_EQ(circularDistance(180_deg, -180_deg), 0_deg);
    BOB_EXPECT_UNIT_T_EQ(circularDistance(-180_deg, 180_deg), 0_deg);
    BOB_EXPECT_UNIT_T_EQ(circularDistance(180_deg, -170_deg), -10_deg);
    BOB_EXPECT_UNIT_T_EQ(circularDistance(-180_deg, 170_deg), 10_deg);
}
