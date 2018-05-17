/*
 * A test for the Display::SimpleDisplay class. Build with make.
 *
 * Invoke with:
 *    ./simpledisplay_test 1          # read from camera
 * Or:
 *    ./simpledisplay_test video.avi  # read from video file
 *
 * If no arguments are given, the default camera device is used.
 *
 * Quit by pressing escape.
 */

#include "video/opencvinput.h"
#include "video/simpledisplay.h"

using namespace Video;

int
main(int argc, char **argv)
{
    SimpleDisplay display;
    if (argc == 1) {
        // if no args supplied, use default webcam
        OpenCVInput cam;
        display.run(cam);
    } else {
        try {
            // if the arg is an int, the user is specifying a camera...
            int dev = std::stoi(argv[1]);
            OpenCVInput cam(dev);
            display.run(cam);
        } catch (std::invalid_argument &e) {
            // ...else it's a filename/URL
            OpenCVInput cam(argv[1]);
            display.run(cam);
        }
    }
}
