// BoB robotics includes
#include "net/server.h"
#ifdef NO_I2C
#include "robots/tank/tank.h"
#else
#include "robots/tank/norbot.h"
#endif

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

#ifdef NO_I2C
    // Print motor commands
    Robots::Tank robot;
#else
    // Use Arduino robot
    Robots::Norbot robot;
#endif

    // Read motor commands from network
    robot.readFromNetwork(*connection);

    // Run server
    connection->run();

    return EXIT_SUCCESS;
}
