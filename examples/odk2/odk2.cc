// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "video/odk2/odk2.h"

// Standard C++ includes
#include <iostream>

using namespace BoBRobotics::Video;

int bobMain(int, char**)
{
    ODK2 odk2;
    BoBRobotics::BackgroundExceptionCatcher catcher;

    cv::namedWindow("Raw frame", cv::WINDOW_NORMAL);
    cv::resizeWindow("Raw frame", odk2.getOutputSize().width * 2, odk2.getOutputSize().height * 2);

    cv::Mat cameraImage;
    while(true) {
        // Check for background exceptions
        catcher.check();

        // Read frames
        odk2.readFrame(cameraImage);

        // Show raw frame
        cv::imshow("Raw frame", cameraImage);

        // Get euler angles from ODK2
        const auto euler = odk2.getEulerAngles();
        std::cout << "Orientation: (" << euler[0].value() << ", " << euler[1].value() << ", "<< euler[2].value() << ") degrees" << std::endl;

        // Pump events and stop if escape is pressed
        if(cv::waitKey(1) == 27) {
            break;
        }
    }

    return EXIT_SUCCESS;
}
