#include "common.h"

// BoB robotics includes
#include "common/string.h"

const std::string testString = "  test  ";
TEST(strTrim, strTrimLeft)
{
    std::string s = testString;
    strTrimLeft(s);
    EXPECT_EQ(s, "test  ");
}

TEST(strTrim, strTrimRight)
{
    std::string s = testString;
    strTrimRight(s);
    EXPECT_EQ(s, "  test");
}
