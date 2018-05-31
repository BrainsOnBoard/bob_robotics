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

// GeNN robotics includes
#include "net/server.h"
#include "robots/motor.h"
#include "video/panoramic.h"

#ifndef NO_I2C_ROBOT
#include "robots/motor_i2c.h"
#endif

using namespace GeNNRobotics;

int
main()
{
    // listen for incoming connection on default port
    Net::Server server;

    // get default camera
    auto cam = Video::getPanoramicCamera();
    cam->streamToNetwork(server); // stream camera over network

#ifdef NO_I2C_ROBOT
    // output motor commands to terminal
    Robots::Motor motor;
#else
    // use Arduino robot
    Robots::MotorI2C motor;
#endif

    // read motor commands from network
    motor.readFromNetwork(server);

    // run server on main thread
    server.run();
}
