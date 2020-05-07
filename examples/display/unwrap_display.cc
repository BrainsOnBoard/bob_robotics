/*
 * A test for the Video::Display class with panoramic camera. Build with make.
 *
 * Quit by pressing escape.
 */

// BoB robotics includes
#include "plog/Log.h"
#include "video/panoramic.h"
#include "video/display.h"

using namespace BoBRobotics::Video;

int bobMain(int, char **)
{
    auto cam = getPanoramicCamera();
    Display display(*cam, {1240, 600});
    display.run();
    return EXIT_SUCCESS;
}
