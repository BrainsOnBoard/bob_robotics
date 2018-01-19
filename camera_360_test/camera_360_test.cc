// Standard C++ includes
#include <iostream>

// Standard C includes
#include <cassert>
#include <cmath>

// Common includes
#include "../common/opencv_optical_flow.h"
#include "../common/opencv_unwrap_360.h"

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

    const cv::Size cameraRes(640, 480);
    const cv::Size unwrapRes(90, 10);
    const unsigned int outputScale = 10;

    // Open video capture device and check it matches desired camera resolution
    cv::VideoCapture capture(device);
    assert(capture.get(cv::CAP_PROP_FRAME_WIDTH) == cameraRes.width);
    assert(capture.get(cv::CAP_PROP_FRAME_HEIGHT) == cameraRes.height);

    // Create unwrapper
    OpenCVUnwrap360 unwrapper(cameraRes, unwrapRes,
                              0.5, 0.416, 0.173, 0.377, -pi);

    // Create optical flow calculator
    OpenCVOpticalFlow opticalFlow(unwrapRes);

    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Create motor
    cv::namedWindow("Unwrapped", CV_WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapRes.width * outputScale,
                     unwrapRes.height * outputScale);

    cv::namedWindow("Original", CV_WINDOW_NORMAL);
    cv::resizeWindow("Original", cameraRes.width, cameraRes.height);

    // Build a velocity filter whose preferred angle is going straighj
    cv::Mat velocityFilter(1, unwrapRes.width, CV_32FC1);
    buildFilter(velocityFilter, 0.0f);

    cv::Mat flowImage(unwrapRes.height * outputScale, unwrapRes.width * outputScale, CV_8UC1);
    cv::Mat flowXSum(1, unwrapRes.width, CV_32FC1);
    cv::Mat flowSum(1, 1, CV_32FC1);

    for(unsigned int i = 0;; i++)
    {
        // Read from camera
        if(!capture.read(originalImage)) {
            return EXIT_FAILURE;
        }

        // Unwrap
        unwrapper.unwrap(originalImage, outputImage);

        // Show frame difference
        cv::imshow("Original", originalImage);
        cv::imshow("Unwrapped", outputImage);

        if(opticalFlow.calculate(outputImage)) {
            // Render optical flow
            opticalFlow.render(flowImage, 10);

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

    return EXIT_SUCCESS;
}


