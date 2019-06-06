// BoB robotics includes
#include "common/logging.h"
#include "common/lm9ds1_imu.h"
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
    const cv::Size unwrapRes(720, 150);

    // Create panoramic camera and suitable unwrapper
    auto cam = getPanoramicCamera();
    auto unwrapper = cam->createUnwrapper(unwrapRes);
    const auto cameraRes = cam->getOutputSize();

    // The cirlce we're unwrapping within represents a 
    // half sphere, therefore it's radius is the arc length
    const double quarterArcLength = unwrapper.m_OuterPixel - unwrapper.m_InnerPixel;
    
    // Cache original centre pixel
    const auto originalCentrePixel = unwrapper.m_CentrePixel;
    
    // Configure IMU accelerometer
    LM9DS1 imu;
    LM9DS1::AccelSettings accelSettings;
    imu.initAccel(accelSettings);

    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Create motor
    cv::namedWindow("Unwrapped", cv::WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapRes.width, unwrapRes.height);

    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::resizeWindow("Original", cameraRes.width / 2, cameraRes.height / 2);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            // Read from camera
            if(!cam->readFrame(originalImage)) {
                return EXIT_FAILURE;
            }
            
            // If there is new accelerometer data available
            if(imu.isAccelAvailable()) {
                // Read sample
                float accelData[3];
                imu.readAccel(accelData);
                
                // Calculate roll and pitch angles (in radians)
                const double roll = atan2(accelData[0], sqrt((accelData[2] * accelData[2]) + (accelData[1] * accelData[1])));
                const double pitch = atan2(accelData[2], sqrt((accelData[1] * accelData[1]) + (accelData[1] * accelData[1])));
        
                // Convert roll and pitch angles to pixels along surface of half sphere and calculate new centre
                unwrapper.m_CentrePixel.x = originalCentrePixel.x - (int)std::round(roll * quarterArcLength);
                unwrapper.m_CentrePixel.y = originalCentrePixel.y - (int)std::round(pitch * quarterArcLength);
                
                // Update unwrapping maps
                unwrapper.updateMaps();
            }

            // Unwrap
            unwrapper.unwrap(originalImage, outputImage);
            
            // Draw cross as stabilised centre
            cv::line(originalImage, 
                     cv::Point(unwrapper.m_CentrePixel.x - 20,
                               unwrapper.m_CentrePixel.y),
                     cv::Point(unwrapper.m_CentrePixel.x + 20,
                               unwrapper.m_CentrePixel.y), 
                     cv::Scalar(0x00, 0xff, 0x00), 2);
            cv::line(originalImage, 
                     cv::Point(unwrapper.m_CentrePixel.x,
                               unwrapper.m_CentrePixel.y - 20),
                     cv::Point(unwrapper.m_CentrePixel.x,
                               unwrapper.m_CentrePixel.y + 20), 
                     cv::Scalar(0x00, 0xff, 0x00), 2);

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


