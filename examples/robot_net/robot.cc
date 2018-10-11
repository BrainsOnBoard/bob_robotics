/*
 * Example program to be run on robot, or locally on a desktop as a server.
 *
 * If you don't want to build in I2C support (e.g. if running on desktop) then
 * make the program with:
 *     NO_I2C_ROBOT=1 make
 *
 * Use the corresponding "computer" program to connect to the server.
 */

// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

// BoB robotics includes
#include "net/server.h"
#include "os/net.h"
#include "robots/tank.h"
#include "video/netsink.h"
#include "video/panoramic.h"

#ifndef NO_I2C_ROBOT
#include "robots/norbot.h"
#endif

// Standard C++ includes
#include <iostream>

using namespace BoBRobotics;

int
main()
{
    try {
        // Enable networking on Windows
        OS::Net::WindowsNetworking net;

        // Listen for incoming connection on default port
        Net::Server server;

        // Get default camera
        auto cam = Video::getPanoramicCamera();

        // Stream camera asynchronously over network
        Video::NetSink netSink(server, *cam);

    #ifdef NO_I2C_ROBOT
        // Output motor commands to terminal
        Robots::Tank motor;
    #else
        // Use Arduino robot
        Robots::Norbot motor;
    #endif

        // Read motor commands from network
        motor.readFromNetwork(server);

        // Run server on main thread
        server.run();
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}
