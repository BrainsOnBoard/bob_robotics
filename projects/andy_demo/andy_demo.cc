// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "common/main.h"
#include "common/map_coordinate.h"
#include "common/stopwatch.h"
#include "video/randominput.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::angle;

void
makeDirectory(const filesystem::path &path)
{
    if (!path.is_directory()) {
        BOB_ASSERT(filesystem::create_directory(path));
    }
}

filesystem::path
getNewFilepath(const filesystem::path folderPath, const std::string &fileExtension)
{
    makeDirectory(folderPath);

    // Put a timestamp in the filename
    std::stringstream ss;
    const auto timer = time(nullptr);
    const auto currentTime = localtime(&timer);
    ss << std::setfill('0')
       << std::setw(2) << currentTime->tm_mday
       << std::setw(2) << currentTime->tm_mon
       << std::setw(2) << currentTime->tm_year - 100
       << "_"
       << std::setw(2) << currentTime->tm_hour
       << std::setw(2) << currentTime->tm_min
       << std::setw(2) << currentTime->tm_sec
       << "_";
    const auto basename = (folderPath / ss.str()).str();

    // Append a number in case we get name collisions
    ss.str(std::string{}); // clear stringstream
    filesystem::path path;
    for (int i = 1; ; i++) {
        ss << i << fileExtension;
        path = basename + ss.str();
        if (!path.exists()) {
            break;
        }
        ss.str(std::string{}); // clear stringstream
    }
    return path;
}

void
writeVideo(std::mutex &logMutex,
           std::atomic_bool &stopFlag,
           const filesystem::path &filepath,
           Video::Input &camera)
{
    {
        std::lock_guard<std::mutex> lock{ logMutex };
        LOGI << "Writing video to: " << filepath;
    }

    auto fps = camera.getFrameRate().value();
    auto size = camera.getOutputSize();
    cv::VideoWriter writer(filepath.str(),
                           cv::VideoWriter::fourcc('P','I','M','1'),
                           fps,
                           size);
    BOB_ASSERT(writer.isOpened());

    // Output video to file as long until stopFlag is set
    cv::Mat fr;
    while (!stopFlag) {
        camera.readFrameSync(fr);
        writer.write(fr);
    }
}

void
driveRobotWithCamera(Video::Input &)
{
    /*
     * **TODO**: Actually do something here :-)
     *
     * I'll stick in the snapshot bot code somewhere like here once
     * I've written a variant for Ackermann robots.
     *          -- AD
     */
}

auto getGPSCoordinates()
{
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_int_distribution<std::mt19937::result_type> dist;
    const auto randDeg = [&](degree_t min, degree_t max)
    {
        const auto val = ((double) dist(rng)) / ((double) dist.max());
        return val * (max - min) + min;
    };
    MapCoordinate::GPSCoordinate coords {
            randDeg(-180_deg, 180_deg),
            randDeg(-90_deg, 90_deg)
    };
    return coords;
}

int
bob_main(int, char **argv)
{
    const auto programPath = filesystem::path{ argv[0] }.parent_path();
    const auto videoFilepath = getNewFilepath(programPath / "videos", ".avi");
    const auto dataFilepath = getNewFilepath(programPath / "data", ".yaml");
    std::atomic_bool stopFlag{ false };

    // So we don't get race conditions when logging on multiple threads
    std::mutex logMutex;

    // Use fake cameras for now
    Video::RandomInput<> camera1({100, 100}), camera2({100, 100});
    camera2.setFrameRate(25_Hz); // So we don't get flooded with frames

    // Write video in background
    std::thread videoWriterThread{ &writeVideo,
                                   std::ref(logMutex),
                                   std::ref(stopFlag),
                                   std::cref(videoFilepath),
                                   std::ref(camera2) };

    // Log data to YAML file
    cv::FileStorage fs{ dataFilepath.str(), cv::FileStorage::READ };
    fs << "{" << "video_filepath" << videoFilepath.str();

    // For now, just kill it after 10 secs
    Stopwatch stopwatch;
    stopwatch.start();
    fs << "coords" << "["; // YAML array
    while (stopwatch.elapsed() < 10s) {
        // Pretend to do something with camera
        driveRobotWithCamera(camera1);

        // Log fake GPS coords
        fs << getGPSCoordinates();
    }
    fs << "]" << "}";

    // Stop writing video
    stopFlag = true;
    videoWriterThread.join();

    return EXIT_SUCCESS;
}
