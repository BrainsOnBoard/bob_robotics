#include "common.h"

// BoB robotics includes
#include "robots/simulated_tank.h"

int
main()
{
    // Our tank-like agent
    Robots::SimulatedTank<> tank;
    constexpr Pose2<meter_t, degree_t> initialPose{ 1_m, 0_m, 180_deg };
    tank.setPose(initialPose);

    runObstacleCircumnavigation(tank, tank);
}
