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
#include "navigation/image_database.h"
#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>


using Eigen::Matrix3d;
using Eigen::Vector3f;

cv::Mat get_cam_image(const unsigned char *image, int width, int height) {
    /* Matrix which contains the BGRA image from Webots' camera */
    cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC4);
    img.data = (uchar *)image;
    return img;
}

// time in [ms] of a simulation step
#define TIME_STEP 16

#define MAX_SPEED 6.28

#define RESIZED_WIDTH 360
#define RESIZED_HEIGHT 90

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// entry point of the controller
int main(int argc, char **argv) {



    Supervisor *robot = new Driver();
    Driver *driver = driver->getDriverInstance();
    webots::GPS *gps = driver->getGPS("GPS");
    webots::InertialUnit *imu = driver->getInertialUnit("IMU");

    Camera *cm;
    Display *disp = driver->getDisplay("display");
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




    // feedback loop: step simulation until an exit event is received
    int current_step = 0;
    while (robot->step(TIME_STEP) != -1) {

        cv::Mat current_image = get_cam_image(cm->getImage(),width, height);
        cv::Mat gray_image, resized;
        cv::Mat processed_image;

        const double speed_value = gps->getSpeed();
        const double *pos = gps->getValues();
        const double *orientation = imu->getRollPitchYaw();
        double yaw = orientation[0];
        double pitch = orientation[1];
        double roll = orientation[2];

        if (current_step % 20 == 0) {
            std::cout << "X: " << pos[0] << "Y: " << pos[1] << "Z: " << pos[2]
                << "Yaw: " << yaw << "Pitch: " << pitch << "Roll: " << roll << std::endl;
        }




        if (!current_image.empty()) {

            cv::cvtColor(current_image, gray_image,cv::COLOR_BGRA2GRAY);
            cv::resize(gray_image, resized, cv::Size(RESIZED_WIDTH ,RESIZED_HEIGHT));
            webots::ImageRef* w_img_ref = disp->imageNew(RESIZED_WIDTH, RESIZED_HEIGHT, resized.data, Display::BGRA);
            disp->imagePaste(w_img_ref,0,0);
            disp->imageDelete(w_img_ref);

            cv::imshow("Gray", resized);
            cv::waitKey(1);

        }


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
        // recording mode
        if (key == key_R) {

            if (isTestRunning) {
                std::cout << " you are currently testing, stop it first" << std::endl;
            }
            else if (recording_mode == false && isRecordRunning == false){
                std::cout << "recording mode enabled" << std::endl;
                recording_mode = true;
                isRecordRunning = true;
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

        current_step++;
    }

    delete driver;
    return 0; //EXIT_SUCCESS
}

