// BoB robotics includes
#include "common/logging.h"
#include "common/bn055_imu.h"

// Standard C includes
#include <cmath>

using namespace BoBRobotics;

int main()
{
    BN055 imu;

    while(true) {
        const auto euler = imu.getVector();
        LOG_INFO << euler[0];
    }
    return EXIT_SUCCESS;
}