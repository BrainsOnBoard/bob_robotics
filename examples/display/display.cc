/*
 * A test for the Video::Display class. Build with make.
 *
 * Invoke with:
 *    ./display_test 1          # read from camera
 * Or:
 *    ./display_test video.avi  # read from video file
 *
 * If no arguments are given, the default camera device is used.
 *
 * Quit by pressing escape.
 */

// Windows headers
#include "os/windows_include.h"

// BoB robotics includes
#include "common/logging.h"
#include "video/opencvinput.h"
#include "video/display.h"

using namespace BoBRobotics::Video;

int
main(int argc, char **argv)
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
            // ...else it's a filename/URL
            OpenCVInput cam(argv[1]);
            Display display(cam);
            display.run();
        }
    }
}
