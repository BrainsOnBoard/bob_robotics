// Standard C++ includes
#include <iostream>

// BoB robotics includes
#include "navigation/perfect_memory.h"

int main()
{
    BoBRobotics::Navigation::PerfectMemory pm(cv::Size(720, 150));
    pm.loadSnapshots("../ant_world_db_creator/ant1_route1");
    std::cout << "Loaded " << pm.getNumSnapshots() << " snapshots" << std::endl;
}