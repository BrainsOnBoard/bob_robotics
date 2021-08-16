// BoB includes
#include "common/background_exception_catcher.h"
#include "common/bn055_imu.h"
#include "common/map_coordinate.h"
#include "common/stopwatch.h"
#include "gps/gps_reader.h"
#include "imgproc/opencv_unwrap_360.h"
#include "navigation/image_database.h"
#include "robots/rc_car_bot.h"
#include "video/panoramic.h"

// Third-party includes
#include "plog/Log.h"

// POSIX includes
#include <unistd.h>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <array>
#include <chrono>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::time;

int
bobMain(int argc, char *argv[])
{
    // setting up
    Robots::PassiveRCCarBot bot;
    GPS::GPSReader gps;
    GPS::GPSData data;
    BN055 imu;

    const millisecond_t runTime = (argc > 1) ? second_t{ std::stod(argv[1]) } : 30_s;
    LOGD << "running for " << runTime;

    /*
     * Use GPS to get time and date, because Jetsons don't remember the system
     * time across boots.
     *
     * As the GPS gives the current time in UTC (and we might be in BST in the
     * UK), we convert to localtime first.
     */
    std::tm currentTime = gps.getCurrentDateTime();
    const auto time = mktime(&currentTime); // get raw time
    currentTime = *localtime(&time);

    // Make a new image database using current time to generate folder name
    Navigation::ImageDatabase database{ currentTime };

    // PixPro defaults to 1440x1440
    auto cam = Video::getPanoramicCamera();
    LOGI << "camera initialised " << cam->getOutputSize();

    BackgroundExceptionCatcher catcher;

    Stopwatch sw;
    std::array<degree_t, 3> angles;

    auto recorder = database.getRouteVideoRecorder(cam->getOutputSize(),
                                                   cam->getFrameRate(),
                                                   "mp4",
                                                   "mp4v",
                                                   { "Pitch [degrees]",
                                                     "Roll [degrees]",
                                                     "Speed",
                                                     "Steering angle [degrees]",
                                                     "UTM zone",
                                                     "GPS quality",
                                                     "Timestamp [ms]" });

    cv::Mat frame;
    GPS::GPSData gpsData;
    catcher.trapSignals();
    sw.start();
    for (auto time = 0_ms; time < runTime;) {
        catcher.check();

        // Read image from camera (synchronously)
        cam->readFrameSync(frame);

        // Sync time to when camera frame was read
        time = sw.elapsed();

        // get imu data
        double roll = NAN, pitch = NAN, yaw = NAN;
        try {
            angles = imu.getEulerAngles();
            yaw = angles[0].value();
            pitch = angles[1].value();
            roll = angles[2].value();
        } catch (std::exception &e) {
            LOGE << "Could not read from IMU: " << e.what();
        }

        // read speed and steering angle from robot
        float botSpeed = NAN;
        degree_t turnAngle = 0_deg;
        try {
            std::tie(botSpeed, turnAngle) = bot.readRemoteControl();
        } catch (std::exception &e) {
            // if we can't read speed or angle, we just write nan values
            LOGE << "Could not read speed and steering angle from robot : " << e.what();
        }

        // get gps data
        gps.read(data);
        const auto &coord = gpsData.coordinate;
        int gpsQual = (int) gpsData.gpsQuality; // gps quality

        // output results
        LOGD << std::setprecision(10)
             << "GPS quality: " << gpsQual
             << " latitude: " << coord.lat.value()
             << " longitude: " << coord.lon.value()
             << " num sats: " << gpsData.numberOfSatellites
             << " time: " << time.value() << std::endl;

        // converting to UTM
        const auto utm = MapCoordinate::latLonToUTM(coord);
        recorder.record(utm.toVector(), degree_t{ yaw }, frame, pitch, roll,
                        botSpeed, turnAngle.value(), utm.zone,
                        (int) gpsData.gpsQuality, time.value());
    }

    // Make sure that written data is actually written to disk before we exit
    ::sync();

    return EXIT_SUCCESS;
}
