#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Supervisor.hpp>
#include <webots/vehicle/Car.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/Device.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include "common/path.h"
#include "common/stopwatch.h"
#include "common/pose.h"
#include "common/bn055_imu.h"
#include "common/background_exception_catcher.h"
#include "navigation/image_database.h"
#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


// Standard C includes
#include <ctime>
#include <chrono>
#include <string.h>

#include <plog/Log.h>
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/DebugOutputAppender.h>
#include "plog/Initializers/RollingFileInitializer.h"

cv::Mat get_cam_image(const unsigned char *image, int width, int height) {
    /* Matrix which contains the BGRA image from Webots' camera */
    cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC4);
    img.data = (uchar *)image;
    return img;
}

// time in [ms] of a simulation step
#define TIME_STEP 50 // frame rate 20 fps

#define MAX_SPEED 6.28

#define RESIZED_WIDTH 360
#define RESIZED_HEIGHT 90

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::time;
// entry point of the controller
int bobMain(int argc, char **argv) {


    plog::init(plog::debug, "Hello.txt"); //

    BoBRobotics::BackgroundExceptionCatcher catcher;
    catcher.trapSignals();
    Supervisor *robot = new Driver();
    Driver *driver = driver->getDriverInstance();
    webots::GPS *gps = driver->getGPS("GPS");
    webots::InertialUnit *imu = driver->getInertialUnit("IMU");

    Camera *cm;
    Display *disp = driver->getDisplay("display");
    disp->setColor(0xFFFFFF);
    Keyboard *kb = new Keyboard();

    cm=driver->getCamera("CAM");
    cm->enable(TIME_STEP);
    kb->enable(TIME_STEP);
    gps->enable(TIME_STEP);
    imu->enable(TIME_STEP);
    int width = cm->getWidth();
    int height = cm->getHeight();



    std::cout << "controller started "  << std::endl;


    double speed = 40;
    double key_1 = 49;
    double key_9 = 59;
    int key_R = 82;
    int key_S = 83;
    int key_T = 84;
    bool recording_mode = false;
    bool test_mode = false;
    int last_keypress = -1;
    bool isTestRunning = false;
    bool isRecordRunning = false;
    bool didRecordStop = true;
    bool didTestStop = true;

    tm currentTime;
    const auto rawTime = std::time(nullptr);
    const auto &systemTime = *gmtime(&rawTime);
    currentTime.tm_mday = systemTime.tm_mday;
    currentTime.tm_mon = systemTime.tm_mon;
    currentTime.tm_year = systemTime.tm_year;

    std::string db_name = "database_entries.csv";
    std::string db_path = "../simulation_databases";

    //BoBRobotics::Navigation::ImageDatabase database{db_path, BoBRobotics::Navigation::DatabaseOptions::Overwrite};
    BoBRobotics::Navigation::ImageDatabase database{currentTime};
    BoBRobotics::Stopwatch sw;
    units::time::millisecond_t elapsed;

    auto recorder = database.createRouteRecorder("png", std::vector<std::string>{"Speed",
                                                           "Steering angle [degrees]",
                                                           "Timestamp [ms]"});



    // feedback loop: step simulation until an exit event is received
    int current_step = 0;
    while (robot->step(TIME_STEP) != -1) {

        cv::Mat current_image = get_cam_image(cm->getImage(),width, height);
        cv::Mat gray_image, resized;
        cv::Mat processed_image;

        const double speed_value = gps->getSpeed();


        const double *pos = gps->getValues();
        const double *orientation = imu->getRollPitchYaw();
        auto yaw = radian_t(orientation[0]);
        auto pitch = radian_t(orientation[1]);
        auto roll = radian_t(orientation[2]);
        auto steering_angle = radian_t(driver->getSteeringAngle());


        double x_pos = pos[0];
        double y_pos = pos[1];
        double z_pos = pos[2];

        if (current_step % 20 == 0) {
            std::cout << "X: " << x_pos << "Y: " << y_pos << "Z: " << z_pos
                << "Yaw: " << degree_t(yaw) << "Pitch: " << degree_t(pitch) << "Roll: " << degree_t(roll) << std::endl;
        }




        if (!current_image.empty()) {

            cv::cvtColor(current_image, gray_image,cv::COLOR_BGRA2GRAY);
            cv::resize(gray_image, resized, cv::Size(RESIZED_WIDTH ,RESIZED_HEIGHT));
            cv::imshow("Gray", resized);
            cv::waitKey(1);
        }

        double const PI = 3.14159265358979323;
        //draw trajectory
        int x = pos[0];
        int y = pos[1];
        float angle_rot = 270.0f; // degrees, not radians
        float radians = angle_rot * (PI / 180.0f); 	// convert degrees to radians
        int nx = x * cos(radians) - y * sin(radians);
        int ny = x * sin(radians) + y * cos(radians);
        disp->drawPixel(int(ny)+250, int(nx)+350);


        int key = kb->getKey(); // first keypress
        // add velocity
        if (key == 315) {
            driver->setCruisingSpeed(speed);
        }
        // setting target speed with the numbers
        if (key >= key_1 && key < key_9) {
            // the speed is 1-9(keys) * 10
            speed = (double)(key-key_1) * (key_9-key_1);
            driver->setCruisingSpeed(speed);
            std::cout << " speed is " << speed << " km/h" << std::endl;
        }
        // q button press
        if (key == 81) {
            std::cout << "quitting, cleaning up..." << std::endl;

            break;
        }

        // recording mode
        if (key == key_R) {

            if (isTestRunning) {
                std::cout << " you are currently testing, stop it first" << std::endl;
            }
            else if (recording_mode == false && isRecordRunning == false){
                std::cout << "recording mode enabled" << std::endl;
                recording_mode = true;
                isRecordRunning = true;
                sw.start();
            }


        }

        // stop recording
        if (key == key_S) {

            if (recording_mode && isRecordRunning == true) {
                std::cout << "stop recording" << std::endl;
                recording_mode = false;
                isRecordRunning = false;

            }
            if (test_mode && isTestRunning) {
                std::cout << "stop testing" << std::endl;
                test_mode = false;
                isTestRunning = false;
            }
        }


        // test mode
        if (key == key_T) {
            if (isRecordRunning) {
                std::cout << " you are currently recording, stop it first" << std::endl;
            }
            else if(test_mode == false && isTestRunning == false) {
                std::cout << "test mode" << std::endl;
                test_mode = true;
                isTestRunning = true;

            }
        }


        if (key == -1) {
            driver->setCruisingSpeed(0.0);
        }

        if (last_keypress != key) {
            if (key != 314 && key != 315 && key != 316) {// exclude arrows from printing
                std::cout << " keypress: " << key << std::endl;
            }
        }
        last_keypress = key;


        int c = kb->getKey(); // second keypress (a key combination's second key)
        if(c>=0) {
            if (c == 314 || key == 314) {
                // turn left
                driver->setSteeringAngle(-0.52);
            }

            else if(c == 316 || key == 316) {
                driver->setSteeringAngle(0.52);
            }
        } else {
            driver->setSteeringAngle(0.0);
        }

        if (recording_mode && isRecordRunning) {
            units::time::millisecond_t elapsed = sw.elapsed();
            std::array<degree_t, 3> attitude{ degree_t(roll), degree_t(pitch), degree_t(yaw) };
            recorder->record({ BoBRobotics::Vector3<meter_t>(meter_t(x_pos),meter_t(y_pos),meter_t(z_pos)), attitude },
                                current_image, speed_value,
                                degree_t(steering_angle).value(),
                                elapsed.value());
            catcher.check();
        }


        current_step++;
    }
    //::sync();
    std::cout << "done" << std::endl;
    //delete driver;
    //delete robot;
    //delete gps;
    //delete imu;
    //delete cm;
    //delete disp;
    //delete kb;
    return 0; //EXIT_SUCCESS
}

