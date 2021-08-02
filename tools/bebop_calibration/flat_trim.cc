// BoB robotics includes
#include "robots/uav/bebop/bebop.h"

// Standard C++ includes
#include <iostream>

int bobMain(int, char **)
{
    BoBRobotics::Robots::Bebop drone;

    std::cout << "Place the drone on a flat surface and press any key to continue."
              << std::endl << std::endl;
    std::cin.ignore();

    std::cout << "Doing flat trim calibration of IMUs..." << std::endl;
    drone.doFlatTrimCalibration();
    std::cout << "Calibration complete." << std::endl;

    return EXIT_SUCCESS;
}