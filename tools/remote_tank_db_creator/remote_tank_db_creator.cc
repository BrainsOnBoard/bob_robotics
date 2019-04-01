// BoB robotics includes
#include "common/arena_object.h"
#include "common/assert.h"
#include "common/background_exception_catcher.h"
#include "common/logging.h"
#include "common/main.h"
#include "common/obstacle_circumnavigation.h"
#include "common/pose.h"
#include "common/read_objects.h"
#include "common/sfml_world.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "net/client.h"
#include "robots/robot_positioner.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"
#include "video/netsource.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include "opencv2/opencv.hpp"

// Standard C++ includes
#include <algorithm>
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

// Take the second highest & lowest x/y values as the limits, so we're always within bounds
auto getArenaLimits()
{
    auto lims = readObjects("arena_limits.yaml").at(0);
    BOB_ASSERT(lims.size() == 4);

    std::sort(lims.begin(), lims.end(), [](const auto &vec1, const auto &vec2) {
        return vec1.x() < vec2.x();
    });
    const auto xLow = lims[1].x();
    const auto xHigh = lims[2].x();

    std::sort(lims.begin(), lims.end(), [](const auto &vec1, const auto &vec2) {
        return vec1.y() < vec2.y();
    });

    const auto offset = 25_cm;
    Vector2<meter_t> low{ xLow + offset, lims[1].y() + offset };
    Vector2<meter_t> high{ xHigh - offset, lims[2].y() - offset };

    return std::make_pair(low, high);
}

int
bob_main(int argc, char **argv)
{
    // Image database parameters
    constexpr auto ImageSeparation = 10_cm;
    const auto arenaLimits = getArenaLimits();
    LOG_INFO << "Arena limits: " << arenaLimits.first << " to " << arenaLimits.second;
    const Navigation::Range xRange{ { arenaLimits.first.x(), arenaLimits.second.x() }, ImageSeparation };
    const Navigation::Range yRange{ { arenaLimits.first.y(), arenaLimits.second.y() }, ImageSeparation };

    // Positioner parameters
    constexpr meter_t StoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t AllowedHeadingError = 5_deg; // the amount of error allowed in the final heading
    constexpr double K1 = 0.6;                      // curveness of the path to the goal
    constexpr double K2 = 5;                        // speed of turning on the curves
    constexpr double Alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double Beta = 0.05;                   // causes to drop velocity if 'k'(curveness) increases
    constexpr meter_t StartSlowingAt = 10_cm;
    constexpr float MinSpeed = 0.4f;
    constexpr float MaxSpeed = 1.0f;

    HID::Joystick joystick;

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);

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

    // Read video over network
    Video::NetSource video(client);
    cv::Mat fr;

    const ObjectVector objects = [&]() {
        if (argc > 1) {
            LOG_INFO << "Loading objects from " << argv[1];
            return readObjects(argv[1]);
        } else {
            return ObjectVector{};
        }
    }();
    CollisionDetector collisionDetector(robotDimensions, objects, 20_cm);

    // Display for robot + objects
    using V = Vector2<meter_t>;
    SFMLWorld<> display{ V{ 5_m, 5_m } };
    auto car = display.createCarAgent(tank.getRobotWidth());
    auto objectShapes = ArenaObject::fromObjects(display, objects, collisionDetector.getResizedObjects());
    std::vector<CrossShape> imagePoints;

    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();
    client.runInBackground();

    LOG_INFO << "Ready";
    const auto check = [&]() {
        std::this_thread::sleep_for(20ms);

        // Check for exceptions on background threads
        catcher.check();

        // Update display
        const auto pose = vicon.getObjectData(0).getPose();
        car.setPose(pose);
        display.update(objectShapes, imagePoints, car);

        // If window is closed, stop
        if (!display.isOpen()) {
            return false;
        }

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

        // Update display
        const auto pose = vicon.getObjectData(0).getPose();
        car.setPose(pose);
        display.update(objectShapes, car);

        // If Y is pressed, start collecting images
        if (joystick.update() && joystick.isPressed(HID::JButton::Y)) {
            if (!vicon.connected()) {
                LOG_ERROR << "Still waiting for Vicon system";
            } else {
                auto viconObject = vicon.getObjectReference("EV3");

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

                Navigation::ImageDatabase database(getNewDatabaseName());
                auto recorder = database.getGridRecorder<true>(xRange, yRange);
                recorder.addMetadata(video, true, false);

                // Check if any of the points would lead to a collision and remove them, if so
                std::vector<std::array<size_t, 3>> goodPositions;
                imagePoints.clear();
                for (auto &gridPosition : recorder.getGridPositions()) {
                    const auto pos = recorder.getPosition(gridPosition);
                    if (collisionDetector.wouldCollide(pos)) {
                        LOG_WARNING << "Would collide at: " << pos << "; will not collect image here";
                    } else {
                        goodPositions.push_back(gridPosition);
                        imagePoints.emplace_back(display.vectorToPixel(pos), 20.f, 2.f, sf::Color::Green);
                    }
                }

                // Iterate through points and save images
                if (!recorder.runAtPositions(avoider, video, goodPositions, check)) {
                    LOG_INFO << "Recording aborted";
                    recorder.abortSave();
                }
            }
        }

        // Control robot with joystick
        tank.drive(joystick);

        std::this_thread::sleep_for(20ms);
    } while (!joystick.isPressed(HID::JButton::B) && display.isOpen());

    return EXIT_SUCCESS;
}
