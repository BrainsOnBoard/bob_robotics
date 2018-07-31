#pragma once

// Standard C includes
#include <cstdio>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
std::string
getRouteDatabaseFilename(const unsigned int routeIndex)
{
    char buf[22];
    snprintf(buf, sizeof(buf), "image_%05d.png", routeIndex);
    return std::string(buf);
}

std::string
getImageDatabaseFilename(const int x, const int y, const int z)
{
    const auto zeroPad = [](const auto value) {
        char num[12];
        snprintf(&num[1], sizeof(num) - 1, "%05d", abs(value));
        num[0] = (value < 0) ? '-' : '+';
        return std::string(num);
    };
    return "image_" + zeroPad(x) + "_" + zeroPad(y) + "_" + zeroPad(z) + ".png";
}
} // BoB robotics