// BoB includes
#include "common/background_exception_catcher.h"
#include "common/bn055_imu.h"
#include "common/gps_reader.h"
#include "common/main.h"
#include "common/map_coordinate.h"
#include "common/stopwatch.h"
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
    Robots::RCCarBot bot;
    GPS::GPSReader gps;
    GPS::GPSData data;
    BN055 imu;
    const int maxTrials = 3;
    int numTrials = maxTrials;

    const millisecond_t runTime = (argc > 1) ? second_t{ std::stod(argv[1]) } : 30_s;
    LOGD << "running for " << runTime;

    // check to see if we get valid coordinates by checking GPS quality
    GPS::GPSData gpsData;
    for (; numTrials > 0; numTrials--) {
        try {
            gps.read(gpsData);
        } catch (GPS::GPSError &e) {
            LOGW << " measuring failed, trying again in 1 second "
                 << "[" << maxTrials - numTrials << "/" << maxTrials << "]";
            std::this_thread::sleep_for(1s);
            continue;
        }

        if (gpsData.gpsQuality != GPS::GPSQuality::INVALID) {
            LOGI << " we have a valid measurement";
            break;
        }
    }

    if (numTrials == 0) {
        LOGW << " There is no valid gps measurement, please try waiting for the survey in to finish and restart the program ";
        return EXIT_FAILURE;
    }

    // Use the system clock to get date (might be wrong!)
    time_t rawtime;
    time(&rawtime);
    std::tm currentTime = *localtime(&rawtime);

    /*
     * Use GPS to get time, because Jetsons don't remember the system time
     * across boots.
     */
    currentTime.tm_hour = gpsData.time.hour;
    currentTime.tm_min = gpsData.time.minute;
    currentTime.tm_sec = gpsData.time.second;

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
            bot.updateState();
            botSpeed = bot.getSpeed();
            turnAngle = bot.getTurningAngle();
        } catch (std::exception &e) {
            // if we can't read speed or angle, we just write nan values
            LOGE << "Could not read speed and steering angle from robot : " << e.what();
        }
        try {

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
        // if there is a gps error write nan values
        catch (GPS::GPSError &e) {
            LOGW << e.what();
            recorder.record(Vector3<millimeter_t>::nan(), degree_t{ yaw },
                            frame, pitch, roll, botSpeed, turnAngle.value(), "",
                            -1, time.value());
        }
    }

    // Make sure that written data is actually written to disk before we exit
    ::sync();

    return EXIT_SUCCESS;
}
