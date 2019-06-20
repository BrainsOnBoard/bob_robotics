#include "common.h"

// BoB robotics includes
#include "common/get_new_path.h"
#include "common/logging.h"
#include "common/macros.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "robots/rc_car_bot.h"
#include "video/panoramic.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

struct GPSStruct {
    GPS::Gps gps{ "/dev/ttyACM0" };
    std::mutex dataMutex;
    GPS::GPSData data;
    bool hasData = false;
    std::atomic<bool> stopFlag{ false };
    std::unique_ptr<std::ofstream> logWriter;
};

void
runGPSThread(GPSStruct &gps)
{
    while (!gps.stopFlag) {
        std::lock_guard<std::mutex> lock{ gps.dataMutex };
        gps.data = gps.gps.getGPSData();
        gps.hasData = true;
    }
}

void
usage(char *programName)
{
    std::cout << programName << " [--no-gps]" << std::endl;
}

int
bob_main(int argc, char **argv)
{
    // Optionally use GPS
    bool useGPS;
    switch (argc) {
    case 1:
        useGPS = false;
        break;
    case 2:
        if (argc == 2 && strcmp(argv[1], "--no-gps") == 0) {
            useGPS = true;
        } else {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    default:
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    bool isTraining = false; // program state
    int imageCount;
    filesystem::path trainingImageFolder;

    // Make a new image database to store images
    const filesystem::path programPath{ argv[0] };
    const auto trainingImageRootPath = programPath.parent_path() / imageFolderPrefix;

    // Initialise hardware
    HID::Joystick joystick;
    auto cam = Video::getPanoramicCamera();
    auto unwrapper = cam->createUnwrapper(cv::Size{ 720, 150 });
    Robots::RCCarBot bot;

    // We may or may not be using GPS
    std::unique_ptr<GPSStruct> gps;
    std::thread gpsThread;
    if (useGPS) {
        gps = std::make_unique<GPSStruct>();
        gpsThread = std::thread(runGPSThread, std::ref(*gps));
    }
    std::unique_ptr<std::ofstream> gpsLogWriter;

    cv::Mat frame, unwrappedFrame;
    Stopwatch imageSpaceTimer;
    do {
        // Poll joystick for events
        if (joystick.update()) {
            if (joystick.isPressed(HID::JButton::Y)) {
                if (isTraining) {
                    LOGW << "Already training; press X button to stop";
                } else {
                    trainingImageFolder = getNewPath(trainingImageRootPath);
                    BOB_ASSERT(filesystem::create_directory(trainingImageFolder));
                    imageCount = 0;

                    // CSV file for logging GPS data
                    if (useGPS) {
                        const auto csvPath = trainingImageFolder / "gps_data.csv";
                        gpsLogWriter = std::make_unique<std::ofstream>(csvPath.str());
                        BOB_ASSERT(gpsLogWriter->good());

                        // Write header
                        (*gpsLogWriter) << "Image num, Lat [deg], Lon [deg], Altitude [m], Velocity [m/s], Satellites, Quality, Time [hh:mm:ss.ms]\n";
                    }

                    isTraining = true;
                }
            } else if (joystick.isPressed(HID::JButton::X)) {
                if (isTraining) {
                    gpsLogWriter.reset(); // Save GPS data
                    isTraining = false;
                } else {
                    LOGW << "Not training!";
                }
            }
        }

        if (isTraining && imageSpaceTimer.elapsed() > 200ms && cam->readGreyscaleFrame(frame)) {
            // Restart timer
            imageSpaceTimer.start();

            // Unwrap panoramic image
            unwrapper.unwrap(frame, unwrappedFrame);

            // Save image to disk
            const auto imagePath = trainingImageRootPath / getImageFilename(++imageCount);
            cv::imwrite(imagePath.str(), unwrappedFrame);

            // Save GPS coords if needed
            if (useGPS) {
                std::lock_guard<std::mutex> lock{ gps->dataMutex };
                (*gpsLogWriter) << imageCount;
                if (gps->hasData) {
                    (*gpsLogWriter) << ", ";
                    writeGPSData(*gpsLogWriter, gps->data);
                } else {
                    // Just write image number, to signal that we have no data
                    (*gpsLogWriter) << "\n";
                }
            }
        }

        // Small delay
        std::this_thread::sleep_for(5ms);
    } while (!joystick.isPressed(HID::JButton::B));

    if (useGPS) {
        LOGD << "Waiting for GPS thread to finish";
        gps->stopFlag = true;
        gpsThread.join();
    }

    return EXIT_SUCCESS;
}
