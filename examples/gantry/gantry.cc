/*
 * Note that there is currently an annoying bug with the gantry's camera.
 * You have to open a TV tuner program *before* this example, so that the
 * TV tuner card is set to PAL B mode (cf. NTSC) otherwise the display
 * will be mangled. There is definitely a way to do this in one of the
 * Windows video APIs (though seemingly not in OpenCV) and I'll add this
 * in when I can be bothered. -- AD
 */

#include "os/windows_include.h"

// BoB robotics includes
#include "common/logging.h"
#include "robots/gantry.h"
#include "video/display.h"
#include "video/opencvinput.h"

// Third-party includes
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::literals;

int
main()
{
    try {
        // Object to interface with gantry robot
        Robots::Gantry gantry;

        // Return gantry to its home position
        LOGI << "Homing gantry...\n";
        gantry.raiseAndHome();
        LOGI << "Gantry homed.\n";

        // Show gantry camera stream
        Video::OpenCVInput cam(0, "gantry");
        cam.setOutputSize({ 720, 576 });
        Video::Display display(cam, { 576, 720 });

        // Move the gantry to the specified coordinates
        LOGI << "Moving gantry...\n";
        gantry.setPosition(500_mm, 500_mm, 0_mm);
        do {
            display.update();
        } while (gantry.isMoving());
        LOGI << "Gantry moved.\n";

        // Print the gantry's current position
        const auto pos = gantry.getPosition();
        LOGI << "Gantry is at: " << pos[0] << ", " << pos[1] << ", "
                  << pos[2] << "\n";
    } catch (std::exception &e) {
        LOGW << "Uncaught exception: " << e.what();
		return 1;
    }
}
