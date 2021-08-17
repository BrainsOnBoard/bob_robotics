#include "auto_controller.h"
#include "robot_recorder.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/macros.h"
#include "robots/ackermann/rc_car_bot.h"

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::math;
using namespace units::velocity;
using namespace units::literals;

int
bobMain(int argc, char **argv)
{
    BOB_ASSERT(argc == 2);

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

        GPS::GPSData gpsData;

        // Get initial position (try to get reading 5 times)
        // (NB: GPS reads are blocking at this point)
        for (int i = 0; i < 5; i++) {
            catcher.check();

            gps.read(gpsData);
            if (gpsData.gpsQuality != GPS::GPSQuality::INVALID) {
                break;
            }
        }
        const auto utmStart = MapCoordinate::latLonToUTM(gpsData.coordinate);

        // Drive forward until we've gone x metres
        MapCoordinate::UTMCoordinate utmEnd{};
        meter_t distance;
        robot.moveForward(InitialDriveSpeed);
        do {
            catcher.check();

            gps.read(gpsData);
            if (gpsData.gpsQuality == GPS::GPSQuality::INVALID) {
                LOGW << "Invalid GPS reading!";
                continue;
            }

            utmEnd = MapCoordinate::latLonToUTM(gpsData.coordinate);
            distance = hypot(utmEnd.easting - utmStart.easting,
                             utmEnd.northing - utmStart.northing);
        } while (distance < InitialDriveDistance);

        // Calculate offset
        const degree_t head = atan2(utmEnd.northing - utmStart.northing,
                                    utmEnd.easting - utmStart.easting);
        return circularDistance(head, imu.getEulerAngles()[0]);
    }();

    // Helper to record sensor data to ImageDatabase
    RobotRecorder recorder{ gps };

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
