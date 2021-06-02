/*
 * A test for the Video::Display class. Build with make.
 *
 * Invoke with:
 *    ./display_test 1          # read from camera
 * Or:
 *    ./display_test video.avi  # read from video file
 * Or:
 *    ./display_test r  		# stream from RPi camera
 *
 * If no arguments are given, the default camera device is used.
 *
 * Quit by pressing escape.
 */

// BoB robotics includes
#include "plog/Log.h"
#include "video/display.h"
#include "video/opencvinput.h"
#include "video/rpi_cam.h"
#ifndef NO_ODK2
#include "video/odk2/odk2.h"
#endif

// Standard C includes
#include <cstring>

using namespace BoBRobotics::Video;

int bobMain(int argc, char **argv)
{
    if (argc == 1) {
        // if no args supplied, use default webcam
        OpenCVInput cam;
        Display display(cam);
        display.run();
    } else {
        try {
            // if the arg is an int, the user is specifying a camera...
            int dev = std::stoi(argv[1]);
            OpenCVInput cam(dev);
            Display display(cam);
            display.run();
        } catch (std::invalid_argument &) {
            // ...else it's a filename/URL/RPiCam
            if (strcmp(argv[1], "r") == 0) {
                // RPicam
                RPiCamera cam(50091);
                Display display(cam);
                display.run();
            }
#ifndef NO_ODK2
            // Otherwise, if it's an Opteran devkit
            else if (strcmp(argv[1], "o") == 0) {
                ODK2 cam;
                Display display(cam);
                display.run();
            }
#endif
            else {
                OpenCVInput cam(argv[1]);
                Display display(cam);
                display.run();
            }
        }
    }

    return EXIT_SUCCESS;
}
