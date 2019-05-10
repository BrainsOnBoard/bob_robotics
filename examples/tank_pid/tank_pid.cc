// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "navigation/read_objects.h"
#include "hid/joystick.h"
#include "robots/control/tank_pid.h"
#include "vicon/udp.h"
#include "viz/plot_agent.h"

// This program can be run locally on the robot or remotely
#ifdef NO_I2C_ROBOT
#include "net/client.h"
#include "robots/tank_netsink.h"
#else
#include "robots/norbot.h"
#endif

// Third-party includes
#include "third_party/matplotlibcpp.h"
#include "third_party/path.h"
#include "third_party/units.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace BoBRobotics;
using namespace units::length;
using namespace units::literals;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::velocity;
using namespace std::literals;
namespace plt = matplotlibcpp;

// For sound
#define PLAY_PATH "/usr/bin/play"
#define SOUND_FILE_PATH "/usr/share/sounds/Oxygen-Im-Message-In.ogg"

void
usage(const char *programName)
{
    std::cout << programName << " [-p path_file.yaml]" << std::endl;
}

void
printGoalStats(const Vector2<millimeter_t> &goal, const Pose3<millimeter_t, radian_t> &robotPosition)
{
    std::cout << "Goal: " << goal << std::endl;
    std::cout << "Distance to goal: "
              << goal.distance2D(robotPosition)
              << std::endl;
}

int
bob_main(int argc, char **argv)
{
    // Parameters
    constexpr meter_t stoppingDistance = 3_cm; // if the robot's distance from goal < stopping dist, robot stops
    constexpr float kp = 0.1f;
    constexpr float ki = 0.1f;
    constexpr float kd = 0.1f;
    constexpr float averageSpeed = 0.5f;

    bool canPlaySound = false;

#ifdef NO_I2C_ROBOT
    // Make connection to robot on default port
    Net::Client client;
    Robots::TankNetSink robot(client);

    if (filesystem::path(PLAY_PATH).exists()) {
        canPlaySound = true;
    } else {
        std::cerr << PLAY_PATH << " not found. Install sox for sounds." << std::endl;
    }
#else
    // Connect to motors over I2C
    Robots::Norbot robot;
#endif

    // Load path from file, if one is given
    std::vector<Vector2<millimeter_t>> goals;
    switch (argc) {
    case 3:
        if (strcmp(argv[1], "-p") != 0) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        goals = std::move(Navigation::readObjects(argv[2]).at(0));
        std::cout << "Path read from " << argv[2] << std::endl;
        break;
    case 1:
        break;
    default:
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    // Default to using (0, 0) as a goal
    if (goals.empty()) {
        goals.emplace_back(0_mm, 0_mm);
    }

    // For iterating through the goals
    decltype(goals)::iterator goalsIter;

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // Drive robot with joystick
    HID::Joystick joystick;
    robot.addJoystick(joystick);

    auto viconObject = vicon.getObjectReference(0);
    auto pid = Robots::createTankPID(robot, viconObject, kp, ki, kd, stoppingDistance, 3_deg, 45_deg, averageSpeed);
    bool runPositioner = false;
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (!pressed) {
            return false;
        }

        switch (button) {
        case HID::JButton::Y:
            runPositioner = !runPositioner;
            if (runPositioner) {
                std::cout << "Starting positioner" << std::endl;

                // Start by aiming for the first goal
                goalsIter = goals.begin();
                pid.moveTo(*goalsIter);

                printGoalStats(*goalsIter, viconObject.getPosition());
            } else {
                robot.stopMoving();
                std::cout << "Stopping positioner" << std::endl;
            }
            return true;
        case HID::JButton::Start:
            std::cout << "Resetting to the first goal" << std::endl;
            goalsIter = goals.begin();
            printGoalStats(*goalsIter, viconObject.getPosition());
            return true;
        default:
            return false;
        }
    });
    joystick.addHandler([&runPositioner](HID::JAxis, float) {
        // If positioner is running, do nothing when joysticks moved
        return runPositioner;
    });

    {
        // Catch exceptions on background threads
        BackgroundExceptionCatcher catcher;
        catcher.trapSignals(); // Trap signals e.g. ctrl+c

#ifdef NO_I2C_ROBOT
        // Read on background thread
        client.runInBackground();
#endif

        std::cout << "Goals: " << std::endl;
        for (auto &goal : goals) {
            std::cout << "\t- " << goal << std::endl;
        }
        std::cout << std::endl
                  << "Press Y to start homing" << std::endl;

        // For plotting goal positions
        std::vector<double> goalX, goalY, currentGoalX(1), currentGoalY(1);
        for (auto &goal : goals) {
            goalX.push_back(goal.x().value());
            goalY.push_back(goal.y().value());
        }
        do {
            // Check for background exceptions
            catcher.check();

            // Poll for joystick events
            joystick.update();

            plt::figure(1);
            plt::clf();

            // Plot goal positions
            plt::plot(goalX, goalY, "b+");

            if (runPositioner) {
                // Plot current goal in green
                currentGoalX[0] = goalsIter->x().value();
                currentGoalY[0] = goalsIter->y().value();
                plt::plot(currentGoalX, currentGoalY, "g+");
            }

            // Plot robot's pose with an arrow
            Viz::plotAgent(viconObject.getPose(), -2000_mm, 2000_mm, -2000_mm, 2000_mm);
            plt::pause(0.025);

            // Get motor commands from positioner, if it's running
            if (runPositioner && !pid.pollPositioner()) {
                // Then we've reached the goal...
                const auto &pose = pid.getPose();
                printGoalStats(*goalsIter, pose);

                // Move on to next goal
                goalsIter++;

                std::cout << "Reached goal "
                          << std::distance(goals.begin(), goalsIter)
                          << "/" << goals.size() << std::endl;
                std::cout << "Final position: " << pose << std::endl;

                robot.stopMoving();
                if (goalsIter == goals.cend()) {
                    runPositioner = false;
                    std::cout << "Reached last goal" << std::endl;
                } else {
                    pid.moveTo(*goalsIter);
                }

                if (canPlaySound) {
                    system(PLAY_PATH " -q " SOUND_FILE_PATH);
                }
            }
        } while (!joystick.isPressed(HID::JButton::B) && plt::fignum_exists(1));
    }

    plt::close();

    return EXIT_SUCCESS;
}
