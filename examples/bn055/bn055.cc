// BoB robotics includes
#include "plog/Log.h"
#include "common/bn055_imu.h"

// Standard C includes
#include <cmath>

using namespace BoBRobotics;

int bobMain(int, char **)
{
    BN055 imu;

    while(true) {
        const auto euler = imu.getEulerAngles();
        LOG_INFO << euler[0];
    }
    return EXIT_SUCCESS;
}