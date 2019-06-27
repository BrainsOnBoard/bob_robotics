#include "common.h"

// BoB robotics includes
#include "common/get_new_path.h"
#include "common/logging.h"
#include "common/macros.h"
#include "common/main.h"
#include "common/threadable.h"
#include "hid/joystick.h"
#include "navigation/perfect_memory.h"
#include "robots/rc_car_bot.h"
#include "video/panoramic.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/simpleopt.h"
#include "third_party/tinydir.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <iostream>
#include <vector>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::math;
using namespace std::literals;

class GPSLogger
  : public Threadable
{
public:
    GPSLogger(GPS::Gps &gps, const std::string &csvPath)
      : m_GPS(gps)
      , m_LogWriter(csvPath)
    {
        LOGI << "Writing GPS data to " << csvPath;
        BOB_ASSERT(m_LogWriter.good());
    }

    virtual ~GPSLogger() override
    {
        stop();
    }

private:
    GPS::Gps &m_GPS;
    std::ofstream m_LogWriter;

protected:
    virtual void runInternal() override
    {
        // Constantly write GPS data to log file in background
        while (isRunning()) {
            auto data = m_GPS.getGPSData();
            writeGPSData(m_LogWriter, data);
        }
    }

};

void
usage(char *programName)
{
    std::cout << programName << " [training image path]\n\n"
              << "(If path is not provided, the last image database will be used.)" << std::endl;
}

int
bob_main(int argc, char **argv)
{
    // Constants
    constexpr float testThrottle = 0.5f; // forwards speed
    const cv::Size imSize{ 360, 75 };

    bool isTesting = false;
    const auto programFolder = filesystem::path{ argv[0] }.parent_path();

    // Parse command-line options
    const CSimpleOpt::SOption options[] = {
        { SO_OPT, "--no-gps", SO_NONE }
    };
    CSimpleOpt args(argc, argv, options);
    if(args.LastError() != SO_SUCCESS || args.FileCount() > 1) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }
    const bool useGPS = args.Next(); // There's only one kind of arg allowed...

    filesystem::path imagesPath;
    if (args.FileCount() > 0) {
        imagesPath = args.File(0);
        if (!imagesPath.is_directory()) {
            LOG_FATAL << "Training image set \"" << imagesPath << "\" not found";
            return EXIT_FAILURE;
        }
    } else { // use latest image database
        tinydir_dir dir;
        BOB_ASSERT(tinydir_open_sorted(&dir, programFolder.str().c_str()) == 0);

        // Go through entries in reverse alphabetical order
        for (int i = dir.n_files - 1; i >= 0; i--) {
            tinydir_file file;
            BOB_ASSERT(tinydir_readfile_n(&dir, &file, i) == 0);
            if (file.is_reg && strncmp(file.name, imageFolderPrefix, strlen(imageFolderPrefix)) == 0) {
                imagesPath = programFolder / file.name;
                break;
            }
        }
        if (imagesPath.empty()) {
            LOG_FATAL << "No training images found";
            return EXIT_FAILURE;
        }

        tinydir_close(&dir);
    }

    // Initialise hardware
    HID::Joystick joystick;
    auto cam = Video::getPanoramicCamera();
    auto unwrapper = cam->createUnwrapper(imSize);
    Robots::RCCarBot bot;
    const degree_t maxTurn = bot.getMaximumTurn();
    std::unique_ptr<GPS::Gps> gps;
    if (useGPS) {
        gps = std::make_unique<GPS::Gps>("/dev/ttyACM0");
    }
    std::unique_ptr<GPSLogger> gpsLogger;

    // Navigation algo
    Navigation::PerfectMemoryRotater<> pm{ imSize };

    // Train with snapshots
    {
        cv::Mat im;
        int numSnapshots = 1;
        for (;; numSnapshots++) {
            const auto filepath = imagesPath / getImageFilename(numSnapshots);
            if (!filepath.exists()) {
                break;
            }

            cv::resize(im, im, imSize);
            pm.train(im); // Train algo
        }
        LOGI << "Loaded " << numSnapshots << " images.";
    }

    cv::Mat frame, unwrappedFrame;
    do {
        if (joystick.update()) {
            if (joystick.isPressed(HID::JButton::Y)) {
                if (isTesting) {
                    bot.stopMoving();
                    gpsLogger.reset(); // Stop logging GPS coordinates
                    isTesting = false;
                } else {
                    if (useGPS) {
                        const auto csvPath = getNewPath(programFolder / "testing" / "gps_data", ".csv");
                        gpsLogger = std::make_unique<GPSLogger>(*gps, csvPath.str());
                    }
                    isTesting = true;
                }
            }

            if (isTesting && cam->readGreyscaleFrame(frame)) {
                unwrapper.unwrap(frame, unwrappedFrame);

                // Get best-matching heading
                const auto res = pm.getHeading(frame);
                const degree_t heading = std::get<0>(res);

                // Drive robot
                bot.move(testThrottle, max(-maxTurn, min(maxTurn, heading)));
            }

            // Small delay
            std::this_thread::sleep_for(5ms);
        }
    } while (!joystick.isPressed(HID::JButton::B));

    return EXIT_SUCCESS;
}
