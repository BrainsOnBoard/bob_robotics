#include "pch.h"
#include <iostream>

int
main()
{
    BoBRobotics::Robots::Gantry gantry;

    std::cout << "Homing gantry" << std::endl;
    gantry.home();
    std::cout << "Homed successfully :-D" << std::endl;
}
