// BoB robotics includes
#include "plog/Log.h"
#include "common/timer.h"
#include "imgproc/opencv_unwrap_360.h"
#include "video/panoramic.h"

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;

int bobMain(int, char **)
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
            cam->readFrameSync(originalImage);

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
        LOGI << "FPS:" << 1000.0 / msPerFrame;
    }

    return EXIT_SUCCESS;
}


