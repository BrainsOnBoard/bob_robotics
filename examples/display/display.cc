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
#include "video/display.h"
#include "video/opencvinput.h"
#include "video/randominput.h"
#include "video/rpi_cam.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <memory>

// Standard C includes
#include <cstring>

using namespace BoBRobotics::Video;

std::unique_ptr<Input>
getCamera(int argc, char **argv)
{
    if (argc == 1) {
        // if no args supplied, use default webcam
        return std::make_unique<OpenCVInput>();
    }

    try {
        // if the arg is an int, the user is specifying a camera...
        return std::make_unique<OpenCVInput>(std::stoi(argv[1]));
    } catch (std::invalid_argument &) {
        // ...else it's a filename/URL/RPiCam
        if (strcmp(argv[1], "rpi") == 0) {
            // RPicam
            return std::make_unique<RPiCamera>(50091);
        }

        if (strcmp(argv[1], "random") == 0) {
            return std::make_unique<RandomInput<>>(cv::Size{ 360, 100 });
        }

        return std::make_unique<OpenCVInput>(argv[1]);
    }
}

int bobMain(int argc, char **argv)
{
    auto cam = getCamera(argc, argv);
    Display display{ *cam };
    display.run();

    return EXIT_SUCCESS;
}
