// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "imgproc/mask.h"
#include "navigation/perfect_memory.h"
#include "video/odk2/odk2.h"

// Standard C++ includes
#include <iostream>

using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Navigation;
using namespace BoBRobotics::Video;

int bobMain(int, char**)
{
    ODK2 odk2;
    BoBRobotics::BackgroundExceptionCatcher catcher;

    PerfectMemory<> pm(odk2.getOutputSize());
    PerfectMemory<> pmMask(odk2.getOutputSize());

    cv::namedWindow("Mask", cv::WINDOW_NORMAL);
    cv::resizeWindow("Mask", odk2.getOutputSize().width * 2, odk2.getOutputSize().height * 2);

    cv::namedWindow("Train", cv::WINDOW_NORMAL);
    cv::resizeWindow("Train", odk2.getOutputSize().width * 2, odk2.getOutputSize().height * 2);

    cv::namedWindow("Test", cv::WINDOW_NORMAL);
    cv::resizeWindow("Test", odk2.getOutputSize().width * 2, odk2.getOutputSize().height * 2);

    // We want to mask out black pixels
    const cv::Scalar maskLowerBound(1, 1, 1);
    const cv::Scalar maskUpperBound(255, 255, 255);

    Mask mask;
    cv::Mat cameraImage;
    cv::Mat cameraImageGrayscale;
    while(true) {
        // Check for background exceptions
        catcher.check();

        // Read frame and convert to grayscale
        odk2.readFrame(cameraImage);
        cv::cvtColor(cameraImage, cameraImageGrayscale, cv::COLOR_BGR2GRAY);

        // Show raw frame
        cv::imshow("Test", cameraImage);

        // Create mask
        mask.set(cameraImage, maskLowerBound, maskUpperBound);
        cv::imshow("Mask", mask.get());

        if(pm.getNumSnapshots() > 0) {
            std::cout << "No mask:" << pm.test(cameraImageGrayscale) << ", mask:" << pmMask.test(cameraImageGrayscale, mask) << std::endl;
        }
        // Get euler angles from ODK2
        //const auto euler = odk2.getEulerAngles();
        //std::cout << "Orientation: (" << euler[0].value() << ", " << euler[1].value() << ", "<< euler[2].value() << ") degrees" << std::endl;

        // Pump events and stop if escape is pressed
        const int key = cv::waitKey(1);
        if(key == 27) {
            break;
        }
        else if(key == 't' && pm.getNumSnapshots() == 0) {
            LOGI << "Training image";
            pmMask.train(cameraImageGrayscale, mask.clone());
            pm.train(cameraImageGrayscale);

            cv::imshow("Train", cameraImage);
        }
    }

    return EXIT_SUCCESS;
}
