// BoB includes
#include "common/background_exception_catcher.h"
#include "common/bn055_imu.h"
#include "common/map_coordinate.h"
#include "common/stopwatch.h"
#include "gps/gps_reader.h"
#include "imgproc/opencv_unwrap_360.h"
#include "navigation/image_database.h"
#include "robots/ackermann/rc_car_bot.h"

#ifdef DUMMY_CAMERA
#include "video/randominput.h"
#else
#include "video/panoramic.h"
#endif

// Third-party includes
#include "plog/Log.h"

// POSIX includes
#include <unistd.h>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <array>
#include <chrono>
#include <memory>
#include <thread>

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
    Robots::Ackermann::PassiveRCCarBot bot;
    GPS::GPSReader gps;
    BN055 imu;

    // We're polling the GPS
    gps.setBlocking(false);

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

#ifdef DUMMY_CAMERA
    Video::RandomInput<> randomInput({ 360, 100 });
    auto *cam = &randomInput;
    LOGW << "USING DUMMY CAMERA!!!!!!!!";
#else
    // PixPro defaults to 1440x1440
    auto cam = Video::getPanoramicCamera();
#endif
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
                                                     "Horizontal dilution [mm]",
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
            std::tie(botSpeed, turnAngle) = bot.readRemoteControl();
        } catch (std::exception &e) {
            // if we can't read speed or angle, we just write nan values
            LOGE << "Could not read speed and steering angle from robot : " << e.what();
        }

        // Poll for GPS data
        if (const auto optData = gps.read()) {
            const auto &gpsData = optData.value();
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
                            (int) gpsData.gpsQuality,
                            millimeter_t{ gpsData.horizontalDilution }.value(),
                            time.value());
        } else {
            /*
             * No new data was available. Indicate this by writing NaNs to the
             * CSV file. We can always estimate these missing values post hoc
             * with interpolation.
             */
            recorder.record(Vector3<millimeter_t>::nan(), degree_t{ yaw },
                frame, pitch, roll, botSpeed, turnAngle.value(), "",
                -1, NAN, time.value());
        }
    }

    // Make sure that written data is actually written to disk before we exit
    ::sync();

    return EXIT_SUCCESS;
}
