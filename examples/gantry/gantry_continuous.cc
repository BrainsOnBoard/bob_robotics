// BoB robotics includes
#include "robots/gantry/gantry.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;
using namespace std::literals;

int bobMain(int, char **)
{
    // Object to interface with gantry robot
    Robots::Gantry gantry;

    // Return gantry to its home position
    LOGI << "Homing gantry...";
    gantry.raiseAndHome();
    LOGI << "Gantry homed.";

    // Start the gantry moving continuously, first in the X direction, then the Y direction
    gantry.continuousMove(X_Axis, 0.05_mps);
    std::this_thread::sleep_for(3s);
    gantry.continuousMove(Y_Axis, 0.05_mps);
    std::this_thread::sleep_for(3s);

    // We now want to change direction. For some reason we have to halt the gantry before we can do this.
    gantry.stopMoving();

    // Move the gantry along two axes in the negative direction.
    gantry.continuousMove(XY_Axis, -0.05_mps);
    std::this_thread::sleep_for(3s);

    return EXIT_SUCCESS;
}
