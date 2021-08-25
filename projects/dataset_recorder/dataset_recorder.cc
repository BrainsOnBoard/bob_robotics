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
#include "third_party/CLI11.hpp"

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

/*
 * Jetsons sadly don't record the time and date while powered off. We can
 * sometimes get the date from the GPS though (also sadly) we don't always
 * get given this information. So let's have a command-line option to force
 * using the system clock. The onus is then on the user to check that the
 * system clock's date is correct.
 *
 * We do however always get *time* information with GPS coordinates, so we
 * can at least rely on that being correct.
 */
const std::tm &
getCurrentDateTime(GPS::GPSReader &gps, const CLI::Option &useSystemClock)
{
    auto currentTime = gps.getCurrentDateTime();

    // We already have current date from GPS
    if (currentTime.tm_year) {
        goto out;
    }

    // Get the date from the system clock (the time still comes from the GPS)
    if (useSystemClock) {
        LOGW << "Using system clock to get current date. This may not be correct!";
        const auto rawTime = std::time(nullptr);
        const auto &systemTime = *gmtime(&rawTime);
        currentTime.tm_mday = systemTime.tm_mday;
        currentTime.tm_mon = systemTime.tm_mon;
        currentTime.tm_year = systemTime.tm_year;
        goto out;
    }

    // See if we get a valid date message in the next 5s...
    {
        Stopwatch sw;
        sw.start();
        while (sw.elapsed() < 5s) {
            currentTime = gps.getCurrentDateTime();
            if (currentTime.tm_year) {
                goto out;
            }
        }
    }

    // ...and if we didn't, give up
    throw std::runtime_error{ "Timed out waiting for date from GPS" };

    // So sue me.
out:
    const auto time = mktime(&currentTime);
    return *localtime(&time);
}

int
bobMain(int argc, char *argv[])
{
    CLI::App app{ "Record data with RC car robot" };
    auto *useSystemClock = app.add_flag("--use-system-clock",
                    "Use the system clock to get current date rather than GPS");
    CLI11_PARSE(app, argc, argv);

    // Time to run data collection for
    millisecond_t runTime;
    switch (app.remaining_size()) {
    case 0:
        runTime = 30_s;
        break;
    case 1:
        runTime = second_t{ std::stod(app.remaining()[0]) };
        break;
    default:
        std::cout << app.help();
        return EXIT_FAILURE;
    }

    // setting up
    Robots::Ackermann::PassiveRCCarBot bot;
    GPS::GPSReader gps;
    BN055 imu;

    // Make a new image database using current time to generate folder name
    Navigation::ImageDatabase database{ getCurrentDateTime(gps, *useSystemClock) };

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
