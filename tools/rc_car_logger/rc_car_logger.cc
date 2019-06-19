// BoB robotics includes
#include "common/assert.h"
#include "common/gps.h"
#include "common/logging.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "video/panoramic.h"

// Third-party includes
#include "third_party/path.h"

// Standard C++ includes
#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>
#include <sstream>

using namespace BoBRobotics;
using namespace std::literals;

filesystem::path
getDataFolder(const filesystem::path &dataBaseFolder)
{
    const std::string root = (dataBaseFolder / "data").str();
    int i = 1;
    std::stringstream ss;
    filesystem::path dataFolder;
    do {
        ss.clear();
        ss << root << "_" << std::setw(3) << std::setfill('0') << i++;
        dataFolder = ss.str();
    } while (dataFolder.exists());
    return dataFolder;
}

void
recordGPS(std::atomic<bool> &stopFlag,
          const filesystem::path &savePath,
          GPS::Gps &gps)
{
    BoBRobotics::GPS::GPSData data;
    BoBRobotics::MapCoordinate::GPSCoordinate coord;
    BoBRobotics::GPS::TimeStamp time;

    cv::FileStorage fs{ (savePath / "gps.yaml").str(), cv::FileStorage::WRITE };

    fs << "gps"
       << "{";
    while (!stopFlag) {
        try {
            data = gps.getGPSData();
            coord = data.coordinate;
            time = data.time;

            fs << "data"
               << "{"
               << "coord"
               << "[" << coord.lat.value() << coord.lon.value() << "]"
               << "time"
               << (std::to_string(time.hour) + ":" + std::to_string(time.minute) + ":" + std::to_string(time.second))
               << "}";
        } catch (BoBRobotics::GPS::GPSError &e) {
            LOG_WARNING << e.what();
        }
    }
    fs << "}";
}

void recordVideo(std::atomic<bool> &stopFlag,
                 const filesystem::path &savePath,
                 Video::Input &camera)
{
    cv::VideoWriter writer{ (savePath / "video.avi").str(),
                        cv::VideoWriter::fourcc('H','2','6','4'),
                        /*fps=*/30,
                        camera.getOutputSize() };
    BOB_ASSERT(writer.isOpened());

    cv::Mat fr;
    while (!stopFlag) {
        camera.readFrameSync(fr);
        writer << fr;
    }
    writer.release();
}

int
bob_main(int, char **argv)
{
    std::cout << "Press any key to begin recording." << std:: endl;
    std::cin.ignore();
    std::cout << "Recording..." << std::endl;

    const auto programFolder = filesystem::path(argv[0]).parent_path();
    const auto dataFolder = getDataFolder(programFolder);
    std::cout << "Saving data to " << dataFolder << std::endl;

#ifdef __linux__
    const char *path = "/dev/ttyACM0"; // path for linux systems
#else
    const char *path = "/dev/cu.usbmodem141401"; // the path for mac is often different!
#endif

    auto camera = Video::getPanoramicCamera();

    BoBRobotics::GPS::Gps gps{ path };

    // if GPS location is invalid, keep trying to get a valid one
    // if failed 10 times we exit
    int numTrials = 10;
    while (gps.getGPSData().gpsQuality == BoBRobotics::GPS::GPSQuality::INVALID && numTrials > 0) {
        std::this_thread::sleep_for(500ms);
        numTrials--;
    }

    if (gps.getGPSData().gpsQuality == BoBRobotics::GPS::GPSQuality::INVALID) {
        LOGE << "Invalid GPS location, please wait until surveying finishes or place the antenna to a better spot";
        return EXIT_FAILURE;
    }

    std::atomic<bool> stopFlag{ false };
    std::thread gpsThread(recordGPS, std::ref(stopFlag), std::cref(dataFolder), std::ref(gps));
    std::thread videoThread(recordVideo, std::ref(stopFlag), std::cref(dataFolder), std::ref(*camera));
    std::cout << "Press any key to exit." << std::endl;
    std::cin.ignore();
    std::cout << "Exiting..." << std::endl;
    gpsThread.join();
    videoThread.join();

    return EXIT_SUCCESS;
}
