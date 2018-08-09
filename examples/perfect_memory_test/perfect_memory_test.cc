// Standard C++ includes
#include <iostream>

// BoB robotics includes
#include "common/timer.h"
#include "navigation/perfect_memory.h"

using namespace BoBRobotics;

int main()
{
    // Class to run perfect memory algorithm
    Navigation::PerfectMemory pm(cv::Size(180, 50));

    // Load snapshots
    pm.loadSnapshotsFromPath("../../ant_world_db_creator/ant1_route1", true);
    std::cout << "Loaded " << pm.getNumSnapshots() << " snapshots" << std::endl << std::endl;

    // Time testing phase
    Timer<> t {"Time taken for testing:"};

    // Treat snapshot #10 as test data
    const auto snap = pm.getSnapshot(10);
    const auto result = pm.getHeading(snap);
    std::cout << "Heading: " << std::get<0>(result) << std::endl;
    std::cout << "Best-matching snapshot: " << std::get<1>(result) << std::endl;
    std::cout << "Difference score: " << std::get<2>(result) << std::endl;
}