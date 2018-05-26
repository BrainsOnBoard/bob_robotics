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
#ifdef NO_I2C_ROBOT
#include "robots/motor_dummy.h"
#else
#include "robots/motor_i2c.h"
#endif
#include "video/panoramic.h"

using namespace GeNNRobotics;

int
main()
{
    // listen for incoming connection on default port
    Net::Server server;

    // get default camera
    auto cam = Video::getPanoramicCamera();
    server.addHandler(*cam.get()); // stream camera over network

#ifdef NO_I2C_ROBOT
    // output motor commands to terminal
    Robots::MotorDummy motor;
#else
    // use Arduino robot
    Robots::MotorI2C motor;
#endif

    // let the motor object handle received motor commands
    server.addHandler(motor);

    // run server on main thread
    server.run();
}
