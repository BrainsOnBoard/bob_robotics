#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Supervisor.hpp>
#include <webots/vehicle/Car.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/Device.hpp>
#include "common/path.h"
#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

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
    Camera *cm;
    Display *disp = driver->getDisplay("display");
    Keyboard *kb = new Keyboard();

    cm=driver->getCamera("CAM");
    cm->enable(TIME_STEP);
    kb->enable(TIME_STEP);
    int width = cm->getWidth();
    int height = cm->getHeight();


    std::cout << "helloszia"  << std::endl;




    // feedback loop: step simulation until an exit event is received
    while (robot->step(TIME_STEP) != -1) {


        cv::Mat current_image = get_cam_image(cm->getImage(),width, height);
        cv::Mat gray_image, resized;
        cv::Mat processed_image;
        if (!current_image.empty()) {


            cv::cvtColor(current_image, gray_image,cv::COLOR_BGRA2GRAY);
            cv::resize(gray_image, resized, cv::Size(RESIZED_WIDTH ,RESIZED_HEIGHT));
            webots::ImageRef* w_img_ref = disp->imageNew(RESIZED_WIDTH, RESIZED_HEIGHT, resized.data, Display::BGRA);
            disp->imagePaste(w_img_ref,0,0);
            disp->imageDelete(w_img_ref);

            cv::imshow("Gray", resized);
            //cv::waitKey(1);

        }

        driver->setCruisingSpeed(15.0);

        int c = kb->getKey();
        if(c>=0) {
            if (c == 314) {

                driver->setSteeringAngle(-0.52);
            }

            else if (c == 316) {
                driver->setSteeringAngle(0.52);
            }


            std::cout << " keypress: " << c << std::endl;
        }
        else {
             driver->setSteeringAngle(0.0);
        }
    }

    delete driver;
    return 0; //EXIT_SUCCESS
}

