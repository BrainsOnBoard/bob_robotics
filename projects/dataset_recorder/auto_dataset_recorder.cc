#include "auto_controller.h"
#include "robot_recorder.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/macros.h"
#include "common/stopwatch.h"
#include "robots/ackermann/rc_car_bot.h"

// Third-party includes
#include "third_party/CLI11.hpp"

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::math;
using namespace units::velocity;
using namespace units::literals;

int
bobMain(int argc, char **argv)
{
    CLI::App app{ "Record data with RC car robot, automatically following previously traversed route" };
    auto *useSystemClock = app.add_flag("--use-system-clock",
                    "Use the system clock to get current date rather than GPS");
    CLI11_PARSE(app, argc, argv);

    constexpr float MaxSpeed = 0.7f;                // car's max speed
    constexpr degree_t MaxTurn = 30_deg;            // car's maximum turning angle
    constexpr millimeter_t LookAheadDistance = 1_m; // lookahead distance
    constexpr millimeter_t StoppingDist = 5_cm;     // car's stopping distance

    // Initialise some of the hardware
    Robots::Ackermann::RCCarBot robot;
    GPS::GPSReader gps;
    BN055 imu;

    // Catch Ctrl+C etc.
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    // Calculate offset between IMU and GPS headings
    const radian_t headingOffset = [&]() {
        constexpr float InitialDriveSpeed = 0.5f;
        constexpr meter_t InitialDriveDistance = 1_m;
        constexpr auto DriveTimeout = 20s;
        constexpr auto ReadDelay = 50ms;

        LOGI << "Calculating robot's position with GPS...";

        // Get initial position (try to get reading 5 times)
        const MapCoordinate::UTMCoordinate utmStart = [&]() {
            LOGI << "Waiting for initial GPS fix";
            for (int i = 0; i < 5; i++) {
                catcher.check();

                const auto data = gps.read();
                if (data && data->gpsQuality != GPS::GPSQuality::INVALID) {
                    return MapCoordinate::latLonToUTM(data->coordinate);
                }

                std::this_thread::sleep_for(ReadDelay);
            }

            throw std::runtime_error{ "Could not get initial GPS fix" };
        }();

        // Drive forward until we've gone x metres
        MapCoordinate::UTMCoordinate utmEnd{};
        meter_t distance;
        LOGI << "Driving forwards...";
        Stopwatch sw;
        sw.start();
        robot.moveForward(InitialDriveSpeed);
        do {
            catcher.check();

            const auto data = gps.read();
            if (data && data->gpsQuality != GPS::GPSQuality::INVALID) {
                utmEnd = MapCoordinate::latLonToUTM(data->coordinate);
                distance = hypot(utmEnd.easting - utmStart.easting,
                                 utmEnd.northing - utmStart.northing);
            } else {
                // Put this additional check in so we don't drive forever
                if (sw.elapsed() > DriveTimeout) {
                    throw std::runtime_error{ "Timed out waiting for valid GPS reading" };
                }
            }

            std::this_thread::sleep_for(ReadDelay);
        } while (distance < InitialDriveDistance);
        robot.stopMoving();

        LOGI << "Successfully calculated heading";

        // Calculate offset
        const degree_t head = atan2(utmEnd.northing - utmStart.northing,
                                    utmEnd.easting - utmStart.easting);
        return circularDistance(head, imu.getEulerAngles()[0]);
    }();

    // Helper to record sensor data to ImageDatabase
    RobotRecorder recorder{ gps, useSystemClock->count() > 0 };

    // **TODO**: We need to actually measure the robot
    constexpr millimeter_t WheelBaseLength = 15_cm;
    AutoController controller{
        argv[1], LookAheadDistance, WheelBaseLength, StoppingDist
    };

    /*
     * Drive the robot along the old route, stopping only when:
     * 1. The robot has reached its destination
     * 2. The PurePursuitController is "stuck"
     * 3. Ctrl+C is pressed
     */
    Pose2<millimeter_t, radian_t> pose;
    do {
        catcher.check();

        // Record sensor values to database
        recorder.step(robot, imu);

        // Get the robot's current pose from GPS + IMU readings
        const auto &gpsData = recorder.getGPSData();
        const auto xy = MapCoordinate::latLonToUTM(gpsData.coordinate).toVector();
        pose.x() = xy.x();
        pose.y() = xy.y();
        pose.yaw() = normaliseAngle180(imu.getEulerAngles()[0] + headingOffset);
    } while (controller.step(pose, robot, MaxSpeed, MaxTurn));

    return EXIT_SUCCESS;
}
