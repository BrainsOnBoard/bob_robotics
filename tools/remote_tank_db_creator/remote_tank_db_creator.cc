// BoB robotics includes
#include "common/main.h"
#include "common/pose.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "net/client.h"
#include "robots/robot_positioner_control.h"
#include "robots/tank_netsink.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// Standard C++ includes
#include <memory>
#include <sstream>
#include <vector>

using namespace BoBRobotics;
using namespace BoBRobotics;
using namespace units::length;
using namespace units::angle;
using namespace units::literals;

constexpr const char *databasePrefix = "database";
const Navigation::Range xrange{ { -1_m, 1_m }, 25_cm };
const Navigation::Range yrange = xrange;

class DatabaseRecorder {
public:
    DatabaseRecorder()
      : m_Database(getNewDatabaseName())
      , m_DatabaseRecorder(m_Database.getGridRecorder(xrange, yrange))
      , m_Goals(m_DatabaseRecorder.getPositions())
      , m_GoalsIter(m_Goals.begin())
    {}

    ~DatabaseRecorder()
    {
        if (m_GoalsIter == m_Goals.begin()) {
            m_DatabaseRecorder.abortSave();
            m_Database.deleteAll();
        }
    }

    bool moveNextGoal()
    {
        return ++m_GoalsIter < m_Goals.end();
    }

    const auto &currentGoal() const
    {
        return *m_GoalsIter;
    }

private:
    Navigation::ImageDatabase m_Database;
    Navigation::ImageDatabase::GridRecorder m_DatabaseRecorder;
    std::vector<Vector3<millimeter_t>> m_Goals;
    decltype(m_Goals)::iterator m_GoalsIter;

    static filesystem::path getNewDatabaseName()
    {
        filesystem::path databaseName;
        int databaseCount = 0;
        do {
            std::stringstream ss;
            ss << databasePrefix << ++databaseCount;
            databaseName = ss.str();
        } while (databaseName.exists());
        return databaseName;
    }
};

int
bob_main(int, char **)
{
    // Wrapper for image database
    std::unique_ptr<DatabaseRecorder> recorder;

    HID::Joystick joystick;

    // Make connection to robot on default port
    Net::Client client;

    // Connect to robot over network
    Robots::TankNetSink tank(client);

    // Positioner parameters
    constexpr meter_t StoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t AllowedHeadingError = 5_deg; // the amount of error allowed in the final heading
    constexpr double K1 = 0.2;                      // curveness of the path to the goal
    constexpr double K2 = 5;                        // speed of turning on the curves
    constexpr double Alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double Beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases

    // Object which drives robot to position
    Robots::RobotPositioner positioner(StoppingDistance,
                                       AllowedHeadingError,
                                       K1,
                                       K2,
                                       Alpha,
                                       Beta);

    // Handles joystick, Vicon system etc.
    Robots::RobotPositionerControl control(tank, positioner, joystick);
    control.setHomingStartedHandler([&recorder, &control]() {
        if (!recorder) {
            recorder = std::make_unique<DatabaseRecorder>();
            control.setGoalPose(recorder->currentGoal());
        }
    });
    control.setHomingStoppedHandler([&recorder, &control](bool reachedGoal) {
        if (reachedGoal) {
            if (recorder->moveNextGoal()) {
                control.setGoalPose(recorder->currentGoal());
                control.startHoming();
                return;
            }

            std::cout << "\nFinished all goals" << std::endl;
        }

        recorder.reset();
    });

    control.run();

    return EXIT_SUCCESS;
}