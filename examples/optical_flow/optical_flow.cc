// BoB robotics includes
#include "common/logging.h"
#include "common/timer.h"
#include "imgproc/opencv_optical_flow.h"
#include "video/panoramic.h"

// Standard C includes
#include <cmath>

using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;

// Anonymous namespace
namespace
{
constexpr float pi = 3.141592653589793238462643383279502884f;

void buildFilter(cv::Mat &filter, float preferredAngle)
{
    // Loop through columns
    float sum = 0.0f;
    for(int x = 0; x < filter.cols; x++) {
        // Convert column to angle
        const float th = (((float)x / (float)filter.cols) * 2.0f * pi) - pi;

        // Write filter with sin of angle
        filter.at<float>(0, x) = sin(th - preferredAngle);
        sum += filter.at<float>(0, x);
        LOGI << x << " = " << filter.at<float>(0, x);
    }
    LOGI << "Sum = " << sum;
}
}   // Anonymous namespace

int main()
{
    const cv::Size unwrapRes(90, 25);
    const unsigned int outputScale = 10;

    // Create panoramic camera and suitable unwrapper
    auto cam = getPanoramicCamera();
    auto unwrapper = cam->createUnwrapper(unwrapRes);
    const auto camRes = cam->getOutputSize();

    // Create optical flow calculator
    OpenCVOpticalFlow opticalFlow(unwrapRes);

    // Create images
    cv::Mat greyscaleInput(camRes, CV_8UC1);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Create windows
    cv::namedWindow("Unwrapped", cv::WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapRes.width * outputScale, unwrapRes.height * outputScale);
    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::resizeWindow("Original", camRes.width, camRes.height);
    cv::namedWindow("Optical flow", cv::WINDOW_NORMAL);
    cv::resizeWindow("Optical flow", unwrapRes.width * outputScale, unwrapRes.height * outputScale);

    // Build a velocity filter whose preferred angle is going straight ahead
    cv::Mat velocityFilter(1, unwrapRes.width, CV_32FC1);
    buildFilter(velocityFilter, 0.0f);

    cv::Mat flowImage(unwrapRes.height * outputScale, unwrapRes.width * outputScale, CV_8UC1);
    cv::Mat flowXSum(1, camRes.width, CV_32FC1);
    cv::Mat flowSum(1, 1, CV_32FC1);

    {
        BoBRobotics::Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            // Read greyscale frame from camera
            if(!cam->readGreyscaleFrame(greyscaleInput)) {
                return EXIT_FAILURE;
            }

            // Unwrap
            unwrapper.unwrap(greyscaleInput, outputImage);

            // Show frame difference
            cv::imshow("Original", greyscaleInput);
            cv::imshow("Unwrapped", outputImage);

            if(opticalFlow.calculate(outputImage)) {
                // Render optical flow
                opticalFlow.render(flowImage, outputScale);

                // Reduce horizontal flow - summing along columns
                cv::reduce(opticalFlow.getFlowX(), flowXSum, 0, cv::REDUCE_SUM);

                // Multiply summed flow by filters
                cv::multiply(flowXSum, velocityFilter, flowXSum);

                // Reduce filtered flow - summing along rows
                cv::reduce(flowXSum, flowSum, 1, cv::REDUCE_SUM);

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
        LOGI << "FPS:" << 1000.0 / msPerFrame;
    }

    return EXIT_SUCCESS;
}


