// BoB robotics includes
#include "common/main.h"
#include "common/logging.h"
#include "common/bn055_imu.h"

// Standard C includes
#include <cmath>

using namespace BoBRobotics;

int bob_main(int, char **)
{
    BN055 imu;

    while(true) {
        const auto euler = imu.getEulerAngles();
        LOG_INFO << euler[0];
    }
    return EXIT_SUCCESS;
}