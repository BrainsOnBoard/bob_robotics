// BoB robotics includes
#include "video/display.h"
#include "video/rpi_cam.h"

using namespace BoBRobotics;

int
main()
{
	// Create camera interface
	Video::RPiCamera cam(50091);

    // Display camera stream until esc pressed
    Video::Display display(cam);
    display.run();
}
