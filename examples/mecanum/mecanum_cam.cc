// BoB robotics includes
#include "hid/joystick.h"
#include "robots/mecanum.h"
#include "common/background_exception_catcher.h"
#include "common/logging.h"
#include "common/timer.h"
#include "common/main.h"
#include "common/macros.h"
#include "imgproc/opencv_unwrap_360.h"
#include "video/panoramic.h"
#include <stdlib.h>
#include <random>

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;

int bob_main(int, char**)
{

    //clear snapshot directory
    system("exec rm -r snapshot_memory/*");
    // CAMERA SET UP
    const cv::Size unwrapRes(720, 200);
    // const unsigned int outputScale = 10;

    // Create panoramic camera and suitable unwrapper
    // use cv::CAP_V4L read from PIX Pro with linux properly
    auto cam = getPanoramicCamera(cv::CAP_V4L);
    auto unwrapper = cam->createUnwrapper(unwrapRes);
    const auto cameraRes = cam->getOutputSize();

    // Create images placeholders
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Joystick lag
    constexpr float joystickDeadzone = 0.25f;

    // Create joystick interface
    HID::Joystick joystick(joystickDeadzone);

    // Create motor interface
    Robots::Mecanum robot;
    BackgroundExceptionCatcher catcher;
    unsigned int index = 0;
    do {
        // Catch exceptions
        catcher.check();

        // Read joystick
        joystick.update();

        // Use joystick to drive motor
        robot.drive(joystick);

        // Read from camera
        cam->readFrame(originalImage);

        // Unwrap
        unwrapper.unwrap(originalImage, outputImage);

        // save image
        const std::string filename = "snapshot_memory/snapshot_" + std::to_string(index++) + ".png" ;
        BOB_ASSERT(cv::imwrite(filename, outputImage));
        LOGI << "Saved snapshot to: " << filename;
        // if (joystick.isDown(HID::JButton::B)){
        //   LOGI << "B pressed";
        // }
    } while(!joystick.isDown(HID::JButton::B));


    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);

    // creat temporary image placeholder
    cv::Mat img(unwrapRes, CV_8UC3);
    // read image from memory, change the orange pixels
    // for black and rewrite it.
    for (int i=0; i<index; i++){
        const std::string filename= "snapshot_memory/snapshot_" + std::to_string(i) + ".png" ;
        img = cv::imread(filename, cv::IMREAD_COLOR );

        // Itearte the image to look for orange pixels
        for(int y=0;y<img.rows;y++)
        {
            for(int x=0;x<img.cols;x++)
            {
                // get pixel
                cv::Vec3b color = img.at<cv::Vec3b>(cv::Point(x,y));
                // Check pixel for orange shades
                if(color[0] <= 50 && color[1] >= 50 && color[2] >= 100)
                {
                  if(distribution(generator) < 0.5){
                    color[0] = 0;
                    color[1] = 0;
                    color[2] = 0;
                  }else{
                    color[0] = 255;
                    color[1] = 255;
                    color[2] = 255;
                  }

                }

                // set pixel
                img.at<cv::Vec3b>(cv::Point(x,y)) = color;
            }
        }

        cv::imwrite(filename, img);
        LOGI << "Changed and saved img:" << filename;
    }

    return 0;
}
