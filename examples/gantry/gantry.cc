#include "os/windows_include.h"

// Standard C++ includes
#include <iostream>

// BoB robotics includes
#include "robots/gantry.h"
#include "video/display.h"
#include "video/opencvinput.h"

using namespace BoBRobotics;

int
main()
{
    try {
		// Object to interface with gantry robot
        Robots::Gantry gantry;

		// Return gantry to its home position
        std::cout << "Homing gantry...\n";
        gantry.home();
        std::cout << "Gantry homed.\n";

		// Show gantry camera stream
		Video::OpenCVInput cam(0, "gantry");
        cam.setOutputSize({720, 576});
        Video::Display display(cam, {576, 720});

		// Move the gantry to the specified coordinates
        std::cout << "Moving gantry...\n";
        gantry.setPosition(500_mm, 500_mm, 0_mm);
        do {
            display.update();
		} while (gantry.isMoving());
        std::cout << "Gantry moved.\n";

		// Print the gantry's current position
        const auto pos = gantry.getPosition<>();
        std::cout << "Gantry is at: " << pos[0] << ", " << pos[1] << ", "
                  << pos[2] << "\n";
    } catch (std::exception &e) {
        std::cout << "Uncaught exception: " << e.what();
    }
}
