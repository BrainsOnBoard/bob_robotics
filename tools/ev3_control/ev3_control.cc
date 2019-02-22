// BoB robotics includes
#include "common/main.h"
#include "net/server.h"
#include "robots/ev3.h"

using namespace BoBRobotics;

int bob_main(int, char **)
{
    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

    // Read motor commands from network
    Robots::EV3 tank;
    tank.readFromNetwork(connection);

    // Run server on main thread
    connection.run();

    return EXIT_SUCCESS;
}
