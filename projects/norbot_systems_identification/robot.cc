// BoB robotics includes
#include "net/server.h"
#include "robots/tank/norbot.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

int bobMain(int, char **)
{
    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

    // Use Arduino robot
    Robots::Tank::Norbot robot;

    // Read motor commands from network
    robot.readFromNetwork(*connection);

    // Run server
    connection->run();

    return EXIT_SUCCESS;
}
