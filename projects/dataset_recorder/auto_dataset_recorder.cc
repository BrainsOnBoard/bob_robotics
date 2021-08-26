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

constexpr float MaxSpeed = 0.7f;                // car's max speed
constexpr degree_t MaxTurn = 30_deg;            // car's maximum turning angle
constexpr millimeter_t LookAheadDistance = 1_m; // lookahead distance
constexpr millimeter_t StoppingDist = 5_cm;     // car's stopping distance

// For quitting the program by moving the remote control
constexpr degree_t TurnAbortThresh = MaxTurn / 2;
constexpr float SpeedAbortThresh = MaxSpeed / 2;

class AutoDatasetRecorderProgram
{
public:
    AutoDatasetRecorderProgram(const Navigation::ImageDatabase &oldDatabase,
                               bool useSystemClock,
                               const BackgroundExceptionCatcher &catcher)
      : m_Catcher{ catcher }
      , m_HeadingOffset{ calculateHeadingOffset() }
      , m_Recorder{ m_GPS, useSystemClock }
      , m_Controller{ oldDatabase, LookAheadDistance,
                      m_Robot.getDistanceBetweenAxes(), StoppingDist }
    {}

    void run()
    {
        /*
         * Drive the robot along the old route, stopping only when:
         * 1. The robot has reached its destination
         * 2. The PurePursuitController is "stuck"
         * 3. Ctrl+C is pressed
         */
        Pose2<millimeter_t, radian_t> pose;
        LOGI << "RECORDING STARTED";
        do {
            checkForErrors();

            // Record sensor values to database
            m_Recorder.step(m_Robot, m_IMU);

            // Get the robot's current pose from GPS + IMU readings
            const auto &gpsData = m_Recorder.getGPSData();
            const auto xy = MapCoordinate::latLonToUTM(gpsData.coordinate).toVector();
            pose.x() = xy.x();
            pose.y() = xy.y();
            pose.yaw() = normaliseAngle180(m_IMU.getEulerAngles()[0] + m_HeadingOffset);
        } while (m_Controller.step(pose, m_Robot, MaxSpeed, MaxTurn));
    }

private:
    Robots::Ackermann::RCCarBot m_Robot;
    GPS::GPSReader m_GPS;
    BN055 m_IMU;
    const BackgroundExceptionCatcher &m_Catcher; // Catch Ctrl+C etc.
    const radian_t m_HeadingOffset;              // Offset between GPS and IMU headings
    RobotRecorder m_Recorder;                    // Helper to record sensor data to ImageDatabase
    AutoController m_Controller;                 // For automatically driving the robot

    void checkForErrors()
    {
        // Background exceptions/signals
        m_Catcher.check();

        // Allow user to abort with remote control
        const auto move = m_Robot.readRemoteControl();
        if (move.first >= SpeedAbortThresh || move.second >= TurnAbortThresh) {
            throw std::runtime_error{ "Control overriden with remote control; aborting" };
        }
    }

    radian_t calculateHeadingOffset()
    {
        constexpr float InitialDriveSpeed = 0.5f;
        constexpr meter_t InitialDriveDistance = 1_m;
        constexpr auto DriveTimeout = 20s;
        constexpr auto ReadDelay = 50ms;

        LOGI << "Calculating robot's position with GPS...";

        // Get initial position (try to get reading 5 times)
        const MapCoordinate::UTMCoordinate utmStart = [&]() {
            LOGI << "Waiting for initial GPS fix";
            for (int i = 0; i < 5; i++) {
                checkForErrors();

                const auto data = m_GPS.read();
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
        m_Robot.moveForward(InitialDriveSpeed);
        do {
            checkForErrors();

            const auto data = m_GPS.read();
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
        m_Robot.stopMoving();

        LOGI << "Successfully calculated heading";

        // Calculate offset
        const degree_t head = atan2(utmEnd.northing - utmStart.northing,
                                    utmEnd.easting - utmStart.easting);
        return circularDistance(head, m_IMU.getEulerAngles()[0]);
    }
};

int
bobMain(int argc, char **argv)
{
    CLI::App app{ "Record data with RC car robot, automatically following previously traversed route" };
    auto *useSystemClock = app.add_flag("--use-system-clock",
                    "Use the system clock to get current date rather than GPS");
    app.allow_extras();
    CLI11_PARSE(app, argc, argv);

    // Need to provide path to old image database
    BOB_ASSERT(app.remaining_size() == 1);

    // Catch Ctrl+C etc.
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    // Run main program
    AutoDatasetRecorderProgram program{ app.remaining()[0],
                                        useSystemClock->count() > 0,
                                        catcher };
    program.run();

    return EXIT_SUCCESS;
}
