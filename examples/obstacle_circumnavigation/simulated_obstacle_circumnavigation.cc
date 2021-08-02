#include "common.h"

// BoB robotics includes
#include "robots/tank/simulated_tank.h"

int bobMain(int, char **)
{
    // Our tank-like agent
    Robots::SimulatedTank<> tank;
    constexpr Pose2<meter_t, degree_t> initialPose{ 0_m, 0_m, 180_deg };
    tank.setPose(initialPose);

    // Object to run obstacle circumnavigator
    auto runner = createRunner(tank, tank);

    // Run program
    while (runner->update());

    return EXIT_SUCCESS;
}
