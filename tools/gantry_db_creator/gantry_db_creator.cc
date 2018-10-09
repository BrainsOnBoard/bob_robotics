#include "os/windows_include.h"

// BoB robotics includes
#include "navigation/image_database.h"
#include "os/keycodes.h"
#include "robots/gantry.h"
#include "video/display.h"
#include "video/opencvinput.h"

// Standard C++ includes
#include <iostream>

using namespace std::literals;
using namespace units::literals;
using namespace BoBRobotics;

int
main()
{
    const auto xstart = 0_mm;
    const auto xend = 100_mm;
    const auto ystart = 0_mm;
    const auto yend = 1700_mm;
    const auto step = 100_mm;
    const auto z = 200_mm;

    try {
		// Open gantry and home it
        Robots::Gantry gantry;
        std::cout << "Homing gantry." << std::endl;
        gantry.raiseAndHome();
        std::cout << "Gantry homed." << std::endl;

		// Get gantry camera
        const cv::Size imSize(720, 576);
        Video::OpenCVInput cam(0, "gantry");
        cam.setOutputSize(imSize);

		// Save images into a folder called gantry
        Navigation::ImageDatabase database("gantry");

        auto x = xstart;
        auto y = ystart;
        cv::Mat frame(imSize, CV_8UC3);
        bool increaseY = true;
        while (true) {
			// While gantry is moving, poll for user keypress
            gantry.setPosition(x, y, z);
            while (gantry.isMoving()) {
                if (cv::waitKeyEx(1) == OS::KeyCodes::Escape) {
                    return 0;
                }
            }

			// Read frame (blocking) and display
			while (!cam.readFrame(frame))
                ;
            cv::imshow("Gantry camera", frame);

			// Write to file with metadata
            database.addImage(frame, x, y, z, 0_deg, false);

			// Get next position
            if (increaseY) {
                if (y < yend) {
                    y += step;
                    continue;
				}
            } else {
                if (y > ystart) {
                    y -= step;
                    continue;
				}
			}
			if (x < xend) {
                x += step;
				increaseY = !increaseY;
            } else {
				// ... otherwise we've finished
                break;
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
    }
}