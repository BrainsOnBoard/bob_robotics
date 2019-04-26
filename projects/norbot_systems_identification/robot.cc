// BoB robotics includes
#include "common/main.h"
#include "net/server.h"
#ifdef NO_I2C
#include "robots/tank.h"
#else
#include "robots/norbot.h"
#endif

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

int
bob_main(int, char **)
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
    robot.readFromNetwork(connection);

    // Run server
    connection.run();

    return EXIT_SUCCESS;
}
