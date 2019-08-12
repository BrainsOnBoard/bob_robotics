// BoB robotics includes
#include "robots/gazebo/camera.h"
#include "video/display.h"

using namespace BoBRobotics;

int
main()
{
    //Initialize Gazebo camera display
    Robots::Gazebo::Camera cam;
    Video::Display display(cam);
    display.run();
}
