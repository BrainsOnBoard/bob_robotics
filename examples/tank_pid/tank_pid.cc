// BoB robotics includes
#include "robots/tank_pid.h"
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "common/read_objects.h"
#include "hid/joystick.h"
#include "vicon/udp.h"

// This program can be run locally on the robot or remotely
#ifdef NO_I2C_ROBOT
#include "net/client.h"
#include "robots/tank_netsink.h"
#else
#include "robots/norbot.h"
#endif

// Third-party includes
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

// For sound
#define PLAY_PATH "/usr/bin/play"
#define SOUND_FILE_PATH "/usr/share/sounds/Oxygen-Im-Message-In.ogg"

void
usage(const char *programName)
{
    std::cout << programName;
#ifdef NO_I2C_ROBOT
    std::cout << " [robot IP]";
#endif
    std::cout << " [-p path_file.yaml]" << std::endl;
}

void
printGoalStats(const Vector2<millimeter_t> &goal, const Vector3<millimeter_t> &robotPosition)
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
    constexpr meter_t stoppingDistance = 5_cm; // if the robot's distance from goal < stopping dist, robot stops
    constexpr float kp = 0.1f;
    constexpr float ki = 0.1f;
    constexpr float kd = 0.1f;
    constexpr float averageSpeed = 0.5f;

    bool canPlaySound = false;

#ifdef NO_I2C_ROBOT
    std::string robotIP;
    if (argc > 1 && strcmp(argv[1], "-p") != 0) {
        // Get robot IP from command-line argument
        robotIP = argv[1];

        // In case there's another argument
        argc--;
        argv++;
    } else {
        // Get robot IP from terminal
        std::cout << "Robot IP [10.0.0.3]: ";
        std::getline(std::cin, robotIP);
        if (robotIP.empty()) {
            robotIP = "10.0.0.3";
        }
    }

    // Make connection to robot on default port
    Net::Client client(robotIP);
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

        goals = std::move(read_objects(argv[2]).at(0));
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
    auto goalsIter = goals.begin();

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // Drive robot with joystick
    HID::Joystick joystick;
    robot.addJoystick(joystick);

    // Throttle the number of motor commands we send
    Stopwatch commandTimer;
    constexpr auto commandSpacing = 100ms;

    Robots::TankPID pid(kp, ki, kd, averageSpeed);
    bool runPositioner = false;
    Stopwatch printTimer;
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (!pressed) {
            return false;
        }

        switch (button) {
        case HID::JButton::Y:
            runPositioner = !runPositioner;
            if (runPositioner) {
                std::cout << "Starting positioner" << std::endl;
                printTimer.start();
                commandTimer.start();
                pid.start();

                printGoalStats(*goalsIter, vicon.getObjectData(0).getPosition());
            } else {
                robot.stopMoving();
                std::cout << "Stopping positioner" << std::endl;
            }
            return true;
        case HID::JButton::Start:
            std::cout << "Resetting to the first goal" << std::endl;
            goalsIter = goals.begin();
            printGoalStats(*goalsIter, vicon.getObjectData(0).getPosition());
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

        while (!joystick.isPressed(HID::JButton::B)) {
            // Check for background exceptions
            catcher.check();

            // Poll for joystick events
            const bool joystickUpdate = joystick.update();

            // Get motor commands from positioner, if it's running
            if (runPositioner) {
                const auto objectData = vicon.getObjectData(0);
                if (objectData.timeSinceReceived() > 10s) {
                    robot.stopMoving();
                    runPositioner = false;
                    std::cerr << "Error: Could not get position from Vicon system\n"
                              << "Stopping trial" << std::endl;
                    continue;
                }

                const auto position = objectData.getPosition();
                const auto distance = goalsIter->distance2D(position);
                if (distance <= stoppingDistance) {
                    // Move on to next goal
                    goalsIter++;

                    // Print stats
                    printGoalStats(*goalsIter, position);
                    std::cout << "Reached goal "
                              << std::distance(goals.begin(), goalsIter)
                              << "/" << goals.size() << std::endl;
                    std::cout << "Final position: " << position << std::endl;

                    robot.stopMoving();
                    if (goalsIter == goals.cend()) {
                        runPositioner = false;
                        std::cout << "Reached last goal" << std::endl;
                    } else {
                        if (canPlaySound) {
                            system(PLAY_PATH " -q " SOUND_FILE_PATH);
                        } else {
                            std::this_thread::sleep_for(1s);
                        }
                    }
                }
                if (runPositioner) {
                    // Drive robot with PID, throttling number of commands sent
                    if (commandTimer.elapsed() > commandSpacing) {
                        commandTimer.start();
                        pid.drive(robot, objectData.getPose(), *goalsIter);
                    }

                    // Print status
                    if (printTimer.elapsed() > 500ms) {
                        printTimer.start();
                        printGoalStats(*goalsIter, position);
                    }
                }
            } else if (!joystickUpdate) {
                std::this_thread::sleep_for(5ms);
            }
        }
    }

    return EXIT_SUCCESS;
}
