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

#include "../videoin/opencvinput.h"
#include "simpledisplay.h"

using namespace VideoIn;

int
main(int argc, char **argv)
{
    OpenCVInput *cam;
    if (argc == 1) {
        // if no args supplied, use default webcam
        cam = new OpenCVInput;
    } else {
        try {
            // if the arg is an int, the user is specifying a camera...
            int dev = std::stoi(argv[1]);
            cam = new OpenCVInput(dev);
        } catch (std::invalid_argument &e) {
            // ...else it's a filename/URL
            cam = new OpenCVInput(argv[1]);
        }
    }

    // show display
    auto pCam = std::unique_ptr<VideoInput>(cam);
    Display::SimpleDisplay<std::unique_ptr<VideoInput> &> display(pCam);
    display.run();

    /*
     * NOTE: If you wanted the display to run on a background thread instead,
     * you would call:
     *     display.startThread();
     *
     * When the SimpleDisplay object is destroyed, the display will close.
     *
     * If you want to join the object's background thread, call:
     *     display.joinThread();
     * */
}
