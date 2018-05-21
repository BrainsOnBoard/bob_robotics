// Standard C++ includes
#include <iostream>

// Standard C includes
#include <cassert>
#include <cmath>

// Common includes
#include "../common/timer.h"
#include "../imgproc/opencv_unwrap_360.h"

using namespace GeNN_Robotics::ImgProc;

int main(int argc, char *argv[])
{
    const unsigned int device = (argc > 1) ? std::atoi(argv[1]) : 0;

    const cv::Size cameraRes(640, 480);
    const cv::Size unwrapRes(90, 10);
    const unsigned int outputScale = 10;

    // Open video capture device and check it matches desired camera resolution
    cv::VideoCapture capture(device);
    assert(capture.get(cv::CAP_PROP_FRAME_WIDTH) == cameraRes.width);
    assert(capture.get(cv::CAP_PROP_FRAME_HEIGHT) == cameraRes.height);

    // Create unwrapper
    OpenCVUnwrap360 unwrapper(cameraRes, unwrapRes,
                              0.5, 0.416, 0.173, 0.377, -3.141592654);

    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Create motor
    cv::namedWindow("Unwrapped", CV_WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapRes.width * outputScale,
                     unwrapRes.height * outputScale);

    cv::namedWindow("Original", CV_WINDOW_NORMAL);
    cv::resizeWindow("Original", cameraRes.width, cameraRes.height);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            // Read from camera
            if(!capture.read(originalImage)) {
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


