// BoB robotics includes
#include "common/path.h"
#include "common/stopwatch.h"
#include "robots/gantry/gantry.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/units.h"

// Standard C++ includes
#include <fstream>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::time;

int bobMain(int, char **)
{
    // Object to interface with gantry robot
    Robots::Gantry gantry;

    // Return gantry to its home position
    LOGI << "Homing gantry...";
    gantry.raiseAndHome();
    LOGI << "Gantry homed.";

    {
        std::ofstream ofs((Path::getRepoPath() / "examples" / "gantry" / "gantry_move.csv").str());
        ofs.exceptions(std::ios::failbit);
        ofs << "t,x,y,z\n";

        // Move the gantry to the specified coordinates
        LOGI << "Moving gantry...";
        gantry.setPosition(500_mm, 500_mm, 0_mm);

        Stopwatch timer;
        timer.start();
        while (gantry.isMoving()) {
            const auto curVelocities = gantry.getVelocity();
            ofs << static_cast<millisecond_t>(timer.elapsed()).value();
            for (size_t i = 0; i < 3; i++) {
                ofs << "," << curVelocities[i].value();
            }
            ofs << "\n";

            std::this_thread::sleep_for(100ms);
        }
        gantry.checkEmergencyButton();

        LOGI << "Gantry moved.";
    }

    // Print the gantry's current position
    const auto pos = gantry.getPosition();
    LOGI << "Gantry is at: " << pos[0] << ", " << pos[1] << ", "
         << pos[2];

    return EXIT_SUCCESS;
}
