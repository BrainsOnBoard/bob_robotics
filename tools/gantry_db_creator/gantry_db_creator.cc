#include "os/windows_include.h"

// BoB robotics includes
#include "common/logging.h"
#include "common/main.h"
#include "navigation/image_database.h"
#include "os/keycodes.h"
#include "robots/gantry.h"
#include "video/display.h"
#include "video/opencvinput.h"

using namespace std::literals;
using namespace units::literals;
using namespace BoBRobotics;

int
bob_main(int, char **)
{
    Navigation::Range xrange({ 0_mm, 100_mm }, 100_mm);
    Navigation::Range yrange({ 0_mm, 1700_mm }, 100_mm);
    const auto z = 200_mm;

	// Get gantry camera
    const cv::Size imSize(720, 576);
    Video::OpenCVInput cam(0, "gantry");
    cam.setOutputSize(imSize);

    // Save images into a folder called gantry
    Navigation::ImageDatabase database("gantry_images", /*overwrite=*/true);
    auto gridRecorder = database.getGridRecorder(xrange, yrange, z);
    auto &metadata = gridRecorder.getMetadataWriter();
    metadata << "camera" << cam
             << "needsUnwrapping" << true
             << "isGreyscale" << false;

    // Open gantry and home it
    Robots::Gantry gantry;
    LOGI << "Homing gantry.";
    gantry.raiseAndHome();
    LOGI << "Gantry homed.";

    cv::Mat frame(imSize, CV_8UC3);
    for (size_t x = 0, y = 0; x < gridRecorder.sizeX();) {
        // Move gantry to next position
        const auto pos = gridRecorder.getPosition({ x, y, 0 });
        gantry.setPosition(pos[0], pos[1], pos[2]);

        // While gantry is moving, poll for user keypress
        while (gantry.isMoving()) {
            if ((cv::waitKeyEx(1) & OS::KeyMask) == OS::KeyCodes::Escape) {
                return EXIT_SUCCESS;
            }
        }

        // Read frame (blocking) and display
        cam.readFrameSync(frame);
        cv::imshow("Gantry camera", frame);

        // Save image
        gridRecorder.record({ x, y, 0 }, frame);

        // If we haven't finished moving along y, move along one more
        if ((x % 2) == 0) {
            if (y < gridRecorder.sizeY() - 1) {
                y++;
                continue;
            }
        } else if (y > 0) {
            // For odd x positions, we decrease y
            y--;
            continue;
        }

        // Otherwise move x
        x++;
    }

    return EXIT_SUCCESS;
}
