// BoB robotics includes
#include "../../common/logging.h"
#include "../../common/timer.h"
#include "../../imgproc/opencv_unwrap_360.h"
#include "../../video/panoramic.h"
#include "../../common/gps.h"
#include "../../third_party/units.h"
#include "../../hid/joystick.h"
#include "../../robots/norbot.h"
#include "../../common/lm9ds1_imu.h"
#include <time.h> 

// Standard C++ includes
#include <iostream>
#include <thread>

// Standard C includes
#include <cmath>

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;
using namespace units::angle;
using namespace units::length;

void gpsThreadFunc(const char *device, std::mutex &mutex, degree_t &latOut, degree_t &lonOut, meter_t &altitudeOut)
{
    //create GPS
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
    const cv::Size unwrapRes(720, 150);
    const unsigned int outputScale = 10;

    // Create panoramic camera and suitable unwrapper
    auto cam = getPanoramicCamera();
    auto unwrapper = cam->createUnwrapper(unwrapRes);
    const auto cameraRes = cam->getOutputSize();

    


    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    constexpr float joystickDeadzone = 0.25f;
    
    // Create joystick interface
    BoBRobotics::HID::Joystick joystick(joystickDeadzone);
    
    // Create motor interface
    BoBRobotics::Robots::Norbot robot;

    // output file for gps 
    std::ofstream file("out.csv");

    // for time
    time_t rawtime;
    struct tm * timeinfo;
    
    degree_t lat,lon;
    meter_t altitude;
    std::mutex gpsMutex;
    std::thread gpsThread(gpsThreadFunc, "/dev/ttyACM0", std::ref(gpsMutex), std::ref(lat), std::ref(lon), std::ref(altitude));

    {
        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            // Read from camera
            if(!cam->readFrame(originalImage)) {
                return EXIT_FAILURE;
            }

            // Unwrap
            unwrapper.unwrap(originalImage, outputImage);

            // save camera
            char filename[255];
            sprintf(filename, "file_%u.jpg", frame);
            cv::imwrite(filename, outputImage);

            time (&rawtime);
            timeinfo = localtime (&rawtime);
            
            {
                std::lock_guard<std::mutex> g(gpsMutex);
                // saving gps coords
                file << lat.value() << "," << lon.value() << "," << robot.getLeft() << "," << robot.getRight() << "," << frame << "," << asctime(timeinfo) << std::endl;
            }
            
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


