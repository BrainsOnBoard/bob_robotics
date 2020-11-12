#include <gtest/gtest.h>

// BoB robotics includes
#include "navigation/image_database.h"

using namespace BoBRobotics::Navigation;

TEST(ImageDatabase, fileNameCompare) {
    const auto check = [](const auto &x, const auto &y) {
        ASSERT_TRUE(ImageDatabase::fileNameCompare(x, y));
        ASSERT_FALSE(ImageDatabase::fileNameCompare(y, x));
    };

    check("a", "b");
    check("image1.png", "image2.png");
    check("image2.png", "image10.png");
    check("image1.png", "image20.png");

    // Check that we fall back on alphabetical comparison if strings don't match
    check("frame2.png", "image1.png");
}
