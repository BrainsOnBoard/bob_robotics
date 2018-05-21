// Standard C++ includes
#include <iostream>

// Standard C includes
#include <cassert>
#include <cmath>

// Common includes
#include "../common/timer.h"
#include "../imgproc/opencv_optical_flow.h"
#include "../imgproc/opencv_unwrap_360.h"
#include "../video/see3cam_cu40.h"

using namespace GeNN_Robotics::ImgProc;
using namespace GeNN_Robotics::Video;

#define USE_SEE3_CAM

// Anonymous namespace
namespace
{
constexpr float pi = 3.141592653589793238462643383279502884f;

void buildFilter(cv::Mat &filter, float preferredAngle)
{
    // Loop through columns
    float sum = 0.0f;
    for(unsigned int x = 0; x < filter.cols; x++) {
        // Convert column to angle
        const float th = (((float)x / (float)filter.cols) * 2.0f * pi) - pi;

        // Write filter with sin of angle
        filter.at<float>(0, x) = sin(th - preferredAngle);
        sum += filter.at<float>(0, x);
        std::cout << x << " = " << filter.at<float>(0, x) << std::endl;
    }
    std::cout << "Sum = " << sum << std::endl;
}
}   // Anonymous namespace

int main(int argc, char *argv[])
{
    const unsigned int device = (argc > 1) ? std::atoi(argv[1]) : 0;

    const cv::Size unwrapRes(90, 10);
    const unsigned int outputScale = 10;

#ifdef USE_SEE3_CAM
    const std::string deviceString = "/dev/video" + std::to_string(device);
    See3CAM_CU40 cam(deviceString, See3CAM_CU40::Resolution::_672x380);

    // Calculate de-bayerered size
    const cv::Size camRes(cam.getWidth() / 2, cam.getHeight() / 2);

    // Create unwrapper to unwrap camera output
    auto unwrapper = cam.createUnwrapper(camRes, unwrapRes);
#else
    // Open video capture device and check it matches desired camera resolution
    cv::VideoCapture capture(device);

    const cv::Size camRes(640, 480);
    assert(capture.get(cv::CAP_PROP_FRAME_WIDTH) == camRes.width);
    assert(capture.get(cv::CAP_PROP_FRAME_HEIGHT) == camRes.height);

    // Create unwrapper
    OpenCVUnwrap360 unwrapper(camRes, unwrapRes,
                              0.5, 0.416, 0.173, 0.377, -180);
#endif

    // Create optical flow calculator
    OpenCVOpticalFlow opticalFlow(unwrapRes);

    // Create images
#ifdef USE_SEE3_CAM
    cv::Mat greyscaleInput(camRes, CV_8UC1);
#else
    cv::Mat rgbInput(camRes, CV_8UC3);
    cv::Mat greyscaleInput(camRes, CV_8UC1);
#endif

    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Create windows
    cv::namedWindow("Unwrapped", CV_WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapRes.width * outputScale, unwrapRes.height * outputScale);
    cv::namedWindow("Original", CV_WINDOW_NORMAL);
    cv::resizeWindow("Original", camRes.width, camRes.height);
    cv::namedWindow("Optical flow", CV_WINDOW_NORMAL);
    cv::resizeWindow("Optical flow", unwrapRes.width * outputScale, unwrapRes.height * outputScale);

    // Build a velocity filter whose preferred angle is going straight ahead
    cv::Mat velocityFilter(1, unwrapRes.width, CV_32FC1);
    buildFilter(velocityFilter, 0.0f);

    cv::Mat flowImage(unwrapRes.height * outputScale, unwrapRes.width * outputScale, CV_8UC1);
    cv::Mat flowXSum(1, camRes.width, CV_32FC1);
    cv::Mat flowSum(1, 1, CV_32FC1);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
#ifdef USE_SEE3_CAM
            // Read directly into greyscale
            if(!cam.captureSuperPixelGreyscale(greyscaleInput)) {
                return EXIT_FAILURE;
            }
#else
            // Read from camera and convert to greyscale
            if(!capture.read(rgbInput)) {
                return EXIT_FAILURE;
            }
            cv::cvtColor(rgbInput, greyscaleInput, CV_BGR2GRAY);
#endif
            // Unwrap
            unwrapper.unwrap(greyscaleInput, outputImage);

            // Show frame difference
            cv::imshow("Original", greyscaleInput);
            cv::imshow("Unwrapped", outputImage);

            if(opticalFlow.calculate(outputImage)) {
                // Render optical flow
                opticalFlow.render(flowImage, outputScale);

                // Reduce horizontal flow - summing along columns
                cv::reduce(opticalFlow.getFlowX(), flowXSum, 0, CV_REDUCE_SUM);

                // Multiply summed flow by filters
                cv::multiply(flowXSum, velocityFilter, flowXSum);

                // Reduce filtered flow - summing along rows
                cv::reduce(flowXSum, flowSum, 1, CV_REDUCE_SUM);

                char textBuffer[256];
                sprintf(textBuffer, "%f", flowSum.at<float>(0, 0));
                cv::putText(flowImage, textBuffer, cv::Point(0, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0xFF, 0xFF, 0xFF));


                // Show flow
                cv::imshow("Optical flow", flowImage);
            }
            if(cv::waitKey(1) == 27) {
                break;
            }
        }

        const double msPerFrame = timer.get() / (double)frame;
        std::cout << "FPS:" << 1000.0 / msPerFrame << std::endl;
    }

    return EXIT_SUCCESS;
}


