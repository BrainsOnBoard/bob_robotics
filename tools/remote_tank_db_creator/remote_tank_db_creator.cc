// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "common/obstacle_circumnavigation.h"
#include "common/pose.h"
#include "common/thread.h"
#include "common/read_objects.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "net/client.h"
#include "robots/robot_positioner.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"
#include "video/randominput.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include "opencv2/opencv.hpp"

// Standard C++ includes
#include <array>
#include <memory>
#include <sstream>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics;
using namespace units::length;
using namespace units::angle;
using namespace units::literals;
using namespace std::literals;

filesystem::path getNewDatabaseName()
{
    filesystem::path databaseName;
    int databaseCount = 0;
    do {
        std::stringstream ss;
        ss << "database" << ++databaseCount;
        databaseName = ss.str();
    } while (databaseName.exists());
    return databaseName;
}

int
bob_main(int, char **)
{
    // Image database parameters
    constexpr Navigation::Range xRange{ { -1_m, 0_m }, 0.5_m };
    constexpr Navigation::Range yRange = xRange;

    // Positioner parameters
    constexpr meter_t StoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t AllowedHeadingError = 5_deg; // the amount of error allowed in the final heading
    constexpr double K1 = 0.6;                      // curveness of the path to the goal
    constexpr double K2 = 5;                        // speed of turning on the curves
    constexpr double Alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double Beta = 0.05;                   // causes to drop velocity if 'k'(curveness) increases
    constexpr meter_t StartSlowingAt = 10_cm;
    constexpr float MinSpeed = 0.2f;
    constexpr float MaxSpeed = 1.0f;

    HID::Joystick joystick;

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    Thread<false> viconWaitThread([&vicon]() {
        while (vicon.getNumObjects() == 0) {
            std::cout << "Waiting for object" << std::endl;
            std::this_thread::sleep_for(1s);
        }
        std::cout << "Got object" << std::endl;
    });
    auto viconObject = vicon.getObjectReference(0);

    // Make connection to robot on default port
    Net::Client client;

    // Connect to robot over network and drive with joystick
    Robots::TankNetSink tank(client);

    // The x and y dimensions of the robot
    using V = Vector2<meter_t>;
    const auto halfWidth = tank.getRobotWidth() / 2;
    const std::array<V, 4> robotDimensions = {
        V{ -halfWidth, halfWidth },
        V{ halfWidth, halfWidth },
        V{ halfWidth, -halfWidth },
        V{ -halfWidth, -halfWidth }
    };

    // Fake video input
    Video::RandomInput<> video({ 720, 360 });
    cv::Mat fr;

    const auto objects = readObjects("objects.yaml");
    CollisionDetector collisionDetector(robotDimensions, objects, 15_cm);

    // Object which drives robot to position
    auto positioner = Robots::createRobotPositioner(tank,
                                                    viconObject,
                                                    StoppingDistance,
                                                    AllowedHeadingError,
                                                    K1,
                                                    K2,
                                                    Alpha,
                                                    Beta,
                                                    StartSlowingAt,
                                                    MinSpeed,
                                                    MaxSpeed);

    // For driving around objects
    auto circum = createObstacleCircumnavigator(tank, viconObject, collisionDetector);
    auto avoider = createObstacleAvoidingPositioner(positioner, circum);

    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();
    client.runInBackground();

    std::cout << "Ready" << std::endl;
    bool collisionMessagePrinted = false;
    const auto check = [&]() {
        std::this_thread::sleep_for(20ms);

        // Check for exceptions on background threads
        catcher.check();

        // If Y or B pressed, abort database collection
        if (joystick.update()) {
            return !(joystick.isPressed(HID::JButton::B) || joystick.isPressed(HID::JButton::Y));
        } else {
            return true;
        }
    };
    do {
        // Check for exceptions on background threads
        catcher.check();

        // If Y is pressed, start collecting images
        if (joystick.update() && joystick.isPressed(HID::JButton::Y)) {
            if (vicon.getNumObjects() == 0) {
                std::cerr << "Error: Still waiting for Vicon system" << std::endl;
            } else {
                Navigation::ImageDatabase database(getNewDatabaseName());
                auto recorder = database.getGridRecorder<true>(xRange, yRange);

                // Check if any of the points would lead to a collision and remove them
                std::vector<std::array<size_t, 3>> goodPositions;
                for (auto &gridPosition : recorder.getGridPositions()) {
                    const auto pos = recorder.getPosition(gridPosition);
                    if (collisionDetector.wouldCollide(pos)) {
                        std::cerr << "Warning: Would collide at: " << pos << "; will not collect image here" << std::endl;
                    } else {
                        goodPositions.push_back(gridPosition);
                    }
                }

                // Iterate through points and save images
                if (!recorder.runAtPositions(avoider, video, goodPositions, check)) {
                    std::cout << "Recording aborted" << std::endl;
                    recorder.abortSave();
                }
            }
        }

        // Control robot with joystick
        tank.drive(joystick);

        // Stop robot colliding with objects
        if (collisionDetector.wouldCollide(viconObject.getPose())) {
            if (!collisionMessagePrinted) {
                tank.stopMoving();
                std::cout << "Collision!" << std::endl;
                collisionMessagePrinted = true;
            }
        } else {
            collisionMessagePrinted = false;
        }

        std::this_thread::sleep_for(20ms);
    } while (!joystick.isPressed(HID::JButton::B));

    return EXIT_SUCCESS;
}
