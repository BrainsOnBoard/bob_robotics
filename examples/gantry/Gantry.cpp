#include "pch.h"
#include <iostream>

int
main()
{
    try {
        BoBRobotics::Robots::Gantry gantry;

        std::cout << "Homing gantry...\n";
        gantry.home();
        std::cout << "Gantry homed.\n";

        std::cout << "Moving gantry...\n";
        gantry.setPosition(500_mm, 500_mm, 200_mm);
        gantry.waitToStopMoving();
        std::cout << "Gantry moved.\n";

        const auto pos = gantry.getPosition();
        std::cout << "Gantry is at: " << pos[0] << ", " << pos[1] << ", "
                  << pos[2] << "\n";
    } catch (std::exception &e) {
        std::cout << "Uncaught exception: " << e.what();
    }
}
