// BoB robotics includes
#include "common/gazebo_node.h"
#include "common/logging.h"
#include "robots/gazebo/omni2d.h"

#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;
using namespace std::literals;

int main()
{
    auto node = getGazeboNode();
    Robots::Gazebo::Omni2D robot(*node, 1_mps);
    LOGI << "Created robot";
    robot.drive(1.f, -1.f, 1.f);
    LOGI << "Driving robot";
    std::this_thread::sleep_for(120s);
    shutdownGazeboNode();
}
