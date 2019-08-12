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
#include "video/gazebocamerainput.h"
#include "video/display.h"

using namespace BoBRobotics::Video;

int
main()
{
    //Initialize Gazebo camera display
    GazeboCameraInput cam("/gazebo/default/differential_drive_robot/camera/link/camera/image");
    Display display(cam);
    display.run();
}
