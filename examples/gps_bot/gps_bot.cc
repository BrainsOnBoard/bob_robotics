// BoB robotics includes
#include "common/logging.h"
#include "common/timer.h"
#include "common/gps.h"
#include "hid/joystick.h"
#include "robots/norbot.h"
#include "third_party/units.h"

// Standard C++ includes
#include <iostream>
#include <thread>

// Standard C includes
#include <cmath>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;

void gpsThreadFunc(const char *device, std::mutex &mutex, degree_t &latOut, degree_t &lonOut, meter_t &altitudeOut)
{
    Gps gps("/dev/ttyACM0");

    std::cout << "GPS created" << std::endl;
    while(true)
    {
        
        // get gps data
        degree_t lat,lon;
        meter_t altitude;
        bool didGetGps = gps.getPosition(lat,lon, altitude);

        {
            std::lock_guard<std::mutex> g(mutex);
            latOut = lat;
            lonOut = lon;
            altitudeOut = altitude;
        }
    }
}
int main()
{
    constexpr float joystickDeadzone = 0.25f;
    
    // Create joystick interface
    BoBRobotics::HID::Joystick joystick(joystickDeadzone);
    
    // Create motor interface
    BoBRobotics::Robots::Norbot robot;

    // output file for gps 
    std::ofstream file("out.csv");

    Timer<> timer("");

    degree_t lat,lon;
    meter_t altitude;
    std::mutex gpsMutex;
    std::thread gpsThread(gpsThreadFunc, "/dev/ttyACM0", std::ref(gpsMutex), std::ref(lat), std::ref(lon), std::ref(altitude));

    {
        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            // Read GPS data and write to file
            {
                std::lock_guard<std::mutex> g(gpsMutex);

                // saving gps coords
                file << std::setprecision(15) << lat.value() << "," << lon.value() << "," << robot.getLeft() << "," << robot.getRight() << timer.get() << std::endl;
            }

            // Read joystick
            joystick.update();

            // Use joystick to drive motor
            robot.drive(joystick);


            if(joystick.isDown(BoBRobotics::HID::JButton::B)) {
                break;
            }

        }

    }

    return EXIT_SUCCESS;
}


