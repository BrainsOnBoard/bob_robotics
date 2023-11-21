// BoB robotics includes
#include "plog/Log.h"
#include "common/bn055_imu.h"

// Standard C includes
#include <cmath>

using namespace BoBRobotics;

int bobMain(int, char **)
{
    BN055 imu;

    bool failed = false;
    while(true) {
        try {
            // Attempt to read IMU
            const auto euler = imu.getEulerAngles();

            // If we had previously failed (but this time it has succeeded because exception wasn't thrown)
            if(failed) {
                LOGE << "Attempting IMU reset";
                imu.setup();
                failed = false;
            }
            // Otherwise, show IMU output
            else {
                LOG_INFO << euler[0];
            }
        } catch (std::exception &e) {
            // If this is the first time reading has failed, show error
            if(!failed) {
                LOGE << "Could not read from IMU: " << e.what();
            }

            // Set fail flag
            failed = true;
        }

    }
    return EXIT_SUCCESS;
}