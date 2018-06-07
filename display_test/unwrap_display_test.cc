/*
 * A test for the Video::Display class with panoramic camera. Build with make.
 *
 * Quit by pressing escape.
 */

#include "video/panoramic.h"
#include "video/display.h"

using namespace GeNNRobotics::Video;

int
main(int argc, char **argv)
{
    auto cam = getPanoramicCamera();
    Display display(cam, 1240, 600);
    display.run();
}
