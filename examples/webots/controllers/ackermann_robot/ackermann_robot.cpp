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

cv::Mat get_cam_image(const unsigned char *image) {
    /* Matrix which contains the BGRA image from Webots' camera */
    cv::Mat img = cv::Mat(cv::Size(1920, 480), CV_8UC4);
    img.data = (uchar *)image;
    return img;
}

// time in [ms] of a simulation step
#define TIME_STEP 64

#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// entry point of the controller
int main(int argc, char **argv) {


  // create the Robot instance.
    //Supervisor *robot = new Driver()
   // Robot *robot = new Robot();
    Supervisor *robot = new Driver();


    //Car *robot = new Car();
    //Driver *driver = new Driver();


    Driver *driver = driver->getDriverInstance();
   // Motor *steering_motor = robot->getMotor("steering_wheel_motor");
   // Motor *rear_motor = robot->getfrom



    Camera *cm;
    //Node *car_node = robot->getFromDef("RC_CAR");
    //Node *steering_node = car_node->getFromProtoDef("CarSteeringWheel");
   // Field *steering_field = steering_node->getField("vehicle steering wheel");

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


        cv::Mat current_image = get_cam_image(cm->getImage());
        cv::Mat gray_image;
        cv::Mat processed_image;
        if (!current_image.empty()) {


            cv::resize(current_image, processed_image, cv::Size(360,90));
            //cv::cvtColor(processed_image, gray_image,cv::COLOR_BGR2GRAY);



            //std::cout << processed_image.data << std::endl;
            webots::ImageRef* w_img_ref = disp->imageNew(360, 90, processed_image.data, Display::RGBA);
            //std::cout << processed_image << std::endl << std::endl;
            disp->imagePaste(w_img_ref,0,0);
            disp->imageDelete(w_img_ref);


        }

        driver->setCruisingSpeed(5.0);

        int c = kb->getKey();
        if(c>=0) {
            if (c == 314) {

                driver->setSteeringAngle(-0.52);
              //  steering_motor->setPosition(-1.5);

                //steering_wheel_field->setSFVec3f(newValue);
                // left
                //driver->setSteeringAngle(-30);
            }

            else if (c == 316) {
              //  steering_motor->setPosition(1.5);
                driver->setSteeringAngle(0.52);
                // driver->setSteeringAngle(30);
                // right
            }


            std::cout << " keypress: " << c << std::endl;
        }
        else {
          //  steering_motor->setPosition(0);
             driver->setSteeringAngle(0.0);
        }


       // auto steering = robot->getDe("vehice steering wheel");



        /* code */


        // read sensors outputs

    }

    delete driver;
    return 0; //EXIT_SUCCESS
}

