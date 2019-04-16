// BoB robotics includes
#include "common/logging.h"
#include "common/timer.h"
#include "imgproc/opencv_unwrap_360.h"
#include "video/panoramic.h"

// Standard C++ includes
#include <iostream>

// Standard C includes
#include <cmath>

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;

int main()
{
    const cv::Size unwrapRes(90, 25);
    const unsigned int outputScale = 10;

    // Create panoramic camera and suitable unwrapper
    auto cam = getPanoramicCamera();
    auto unwrapper = cam->createUnwrapper(unwrapRes);
    const auto cameraRes = cam->getOutputSize();

    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Create motor
    cv::namedWindow("Unwrapped", cv::WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapRes.width * outputScale,
                     unwrapRes.height * outputScale);

    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::resizeWindow("Original", cameraRes.width, cameraRes.height);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            // Read from camera
            if(!cam->readFrame(originalImage)) {
                return EXIT_FAILURE;
            }

            // Unwrap
            unwrapper.unwrap(originalImage, outputImage);

            // Show frame difference
            cv::imshow("Original", originalImage);
            cv::imshow("Unwrapped", outputImage);

            if(cv::waitKey(1) == 27) {
                break;
            }
        }

        const double msPerFrame = timer.get() / (double)frame;
        std::cout << "FPS:" << 1000.0 / msPerFrame << std::endl;
    }

    return EXIT_SUCCESS;
}


