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

using namespace BoBRobotics;

int
main()
{
    // start networking API on Windows
    WSAStartup();

    // listen for incoming connection on default port
    Net::Server server;

    // get default camera
    auto cam = Video::getPanoramicCamera();

    // stream camera asynchronously over network
    Video::NetSink netSink(server, *cam);

#ifdef NO_I2C_ROBOT
    // output motor commands to terminal
    Robots::Tank motor;
#else
    // use Arduino robot
    Robots::Norbot motor;
#endif

    // read motor commands from network
    motor.readFromNetwork(server);

    // run server on main thread
    server.run();

    // shutdown networking API on Windows
    WSACleanup();
}
