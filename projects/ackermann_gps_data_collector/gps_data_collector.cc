// standard includes
#include<string>
#include<vector>
#include <thread>

// BoB includes
#include "../../common/gps.h"
#include "../../common/logging.h"
#include "../../common/map_coordinate.h"

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;

int main()
{

    // setting up gps ---------------
    const char *path_linux = "/dev/ttyACM0";       // path for linux systems
    BoBRobotics::GPS::Gps gps;
    gps.connect(path_linux);
    const int maxTrials = 20;
    int numTrials = maxTrials;
    while (numTrials > 0) {
        try {
            if (gps.getGPSData().gpsQuality != BoBRobotics::GPS::GPSQuality::INVALID) {
                std::cout << " we have a valid measurement" << std::endl;
                break;
            }
        } catch(...) {
            std::cout << " measuring failed, trying again in 1 second " << "[" << maxTrials - numTrials << "/" << maxTrials << "]" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            numTrials--;
        }    
    }  
    if (numTrials == 0) {
        std::cout << " There is no valid gps measurement, please try waiting for the survey in to finish and restart the program " << std::endl;
        exit(1);
    }
    ///------------------------------

    // setting up the camera ----------------------------
    const cv::Size unwrapRes(90, 25);
    const unsigned int outputScale = 10;

    // Create panoramic camera and suitable unwrapper
    auto cam = getPanoramicCamera();
    auto unwrapper = cam->createUnwrapper(unwrapRes);
    const auto cameraRes = cam->getOutputSize();

    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);
    //---------------------------------------------------

    int i = 0;
    // print for 100 timestep
    while (i < 100) {
        try {
            BoBRobotics::GPS::GPSData data = gps.getGPSData();
            BoBRobotics::MapCoordinate::GPSCoordinate coord = data.coordinate;
            BoBRobotics::GPS::TimeStamp time = data.time;

            std::cout << "latitude: " << coord.lat.value()<< " longitude: " <<  coord.lon.value() << "[ "
                        <<  time.hour << ":" << time.minute << ":" <<  time.second << ":" << time.millisecond << "] " << std::endl;


            // Read from camera
            if(!cam->readFrame(originalImage)) {
                return EXIT_FAILURE;
            }
            // Unwrap
            unwrapper.unwrap(originalImage, outputImage);
            ostringstream fileString;
            fileString << "image" << i << ".jpg" << std::endl;
            cv::imwrite(fileString, outputImage);

            i++;

        } catch(BoBRobotics::GPS::GPSError &e) { LOG_WARNING << e.what(); }
        // The gps receiver is set to 5HZ
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }



    return 0;

}

