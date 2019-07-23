// BoB robotics includes
#include "antworld/camera.h"
#include "common/get_new_path.h"
#include "common/logging.h"
#include "common/macros.h"
#include "common/main.h"
#include "common/map_coordinate.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "navigation/perfect_memory.h"
#include "robots/simulated_ackermann.h"
#include "video/randominput.h"
#include "viz/sfml_world/arena_object.h"
#include "viz/sfml_world/sfml_world.h"

// Third-party includes
#include "third_party/path.h"

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <chrono>
#include <memory>
#include <sstream>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::velocity;

void
makeDirectory(const filesystem::path &path)
{
    if (!path.is_directory()) {
        BOB_ASSERT(filesystem::create_directory(path));
    }
}

enum class ExperimentState
{
    None,
    Training,
    TrainingFinished,
    Testing,
    TestingFinished
};

int
bob_main(int, char **argv)
{
    LOGI << "Joystick controls:\n- Move with joysticks\n- Y: Toggle training mode\n- X: Toggle testing mode\n- B: Quit";

    const auto programPath = filesystem::path{ argv[0] }.parent_path();

    constexpr float testingThrottle = 0.5f;
    ExperimentState state = ExperimentState::None;
    std::unique_ptr<Navigation::ImageDatabase> trainingImages;
    std::unique_ptr<Navigation::ImageDatabase::RouteRecorder> recorder;
    Stopwatch snapshotTimer;

    const cv::Size RenderSize{ 180, 35 };
    const meter_t AntHeight = 1_cm;

    HID::Joystick joystick;

    // Navigation algorithm
    Navigation::PerfectMemoryRotater<> pm{ RenderSize };

    // We need to do this before we create the Renderer
    auto window = AntWorld::Camera::initialiseWindow(RenderSize);

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    auto &world = renderer.getWorld();
    const auto objectsGL = world.load(filesystem::path(argv[0]).parent_path() / "../../resources/antworld/world5000_gray.bin",
               { 0.0f, 1.0f, 0.0f },
               { 0.898f, 0.718f, 0.353f });
    const auto minBound = world.getMinBound();
    const auto maxBound = world.getMaxBound();

    // SFML visualisation
    Viz::SFMLWorld display{ minBound, maxBound };

    // Lines for outward/return path
    auto lineTraining = display.createLineStrip(sf::Color::Blue);
    auto lineTesting = display.createLineStrip(sf::Color::Red);

    // Get objects
    std::vector<sf::ConvexShape> objects;
    objects.reserve(objectsGL.size() / (3 * 3)); // Number of triangles
    for (auto it = objectsGL.begin() + 18; it < objectsGL.end(); it += 3 * 3) {
        // Create shape object
        objects.emplace_back(3);
        auto &triangle = objects.back();
        triangle.setFillColor(sf::Color{ 0x008800FF });

        for (size_t i = 0; i < 3; i++) {
            // Set position of vertices
            Vector2<meter_t> vector{ meter_t{ *(it + (3 * i)) },
                                     meter_t{ *(it + (3 * i) + 1) }};
            triangle.setPoint(i, display.vectorToPixel(vector));
        }
    }

    // Create agent and put in the centre of the world
    AntWorld::Camera antCamera(window.get(), renderer, RenderSize);
    antCamera.setPosition((maxBound[0] - minBound[0]) / 2, (maxBound[1] - minBound[1]) / 2, AntHeight);

    // Simulated robot
    Robots::SimulatedAckermann robot{ 1_mph, 20_cm, 10_cm };
    robot.addJoystick(joystick);

    // Visualisation of robot position
    auto carAgent = display.createCarAgent(robot.getDistanceBetweenAxis());

    // Log position to YAML file
    const auto dataDir = programPath / "data";
    if (!dataDir.is_directory()) {
        BOB_ASSERT(filesystem::create_directory(dataDir));
    }
    const auto dataFilepath = getNewPath(dataDir / "data", ".yaml");
    LOGI << "Saving coordinates to " << dataFilepath;
    cv::FileStorage fs{ dataFilepath.str(), cv::FileStorage::WRITE };
    BOB_ASSERT(fs.isOpened());

    // **HACK**
    auto stopTraining = [&]() {
        // Save metadata
        recorder.reset();

        // Train algorithm with images
        pm.trainRoute(*trainingImages, /*resizeImages=*/false);

        LOGI << "Training finished (" << trainingImages->size() << " images saved)";
    };

    cv::Mat fr;
    Stopwatch stopwatch;
    do {
        joystick.update();
        if (joystick.isPressed(HID::JButton::Y)) { // Toggle training
            switch (state) {
            case ExperimentState::None:
            case ExperimentState::TestingFinished: {
                // Clear lines on screen
                lineTraining.clear();
                lineTesting.clear();

                // Start recording image database
                const auto dbPath = getNewPath(programPath / "snapshots");
                LOGI << "Saving training images to " << dbPath;
                trainingImages = std::make_unique<Navigation::ImageDatabase>(dbPath);
                recorder = trainingImages->getRouteRecorder();
                snapshotTimer.start();

                state = ExperimentState::Training;
            } break;
            case ExperimentState::Training:
                stopTraining();
                if (trainingImages->empty()) {
                    // ... then we didn't actually do any training
                    state = ExperimentState::None;
                } else {
                    state = ExperimentState::TrainingFinished;
                }
                break;
            default:
                break;
            }
        } else if (joystick.isPressed(HID::JButton::X)) { // Toggle testing
            switch (state) {
            case ExperimentState::None:
                LOGE << "No training data!";
                break;
            case ExperimentState::Training:
                stopTraining();
                // fall through
            case ExperimentState::TestingFinished:
            case ExperimentState::TrainingFinished:
                lineTesting.clear();
                robot.moveForward(testingThrottle);
                state = ExperimentState::Testing;
                LOGI << "Starting testing";
                break;
            case ExperimentState::Testing:
                robot.stopMoving();
                state = ExperimentState::TestingFinished;
                break;
            }
        }

        const auto pose = robot.getPose();

        // Render in visualisation
        carAgent.setPose(pose);
        display.update(objects, lineTraining, lineTesting, carAgent);
        if (display.mouseClicked()) {
            Vector3<meter_t> pos = display.mouseClickPosition();
            pos.z() = robot.getPosition().z();
            robot.setPose(pos);
        }

        // Rerender view of ant world
        antCamera.setPose(pose);
        antCamera.update();

        switch (state) {
        case ExperimentState::Training:
            if (snapshotTimer.elapsed() > 100ms) {
                snapshotTimer.start(); // reset

                // Save snapshot
                antCamera.readGreyscaleFrame(fr);
                recorder->record(pose, pose.yaw(), fr);

                // Draw path on screen
                lineTraining.append(pose);
            }
            break;
        case ExperimentState::Testing:
            // Read frame
            antCamera.readGreyscaleFrame(fr);

            // Get best-matching heading
            degree_t heading;
            size_t bestSnapshot;
            std::tie(heading, bestSnapshot, std::ignore, std::ignore) = pm.getHeading(fr);
            LOGI << "Heading: " << heading << "; best snapshot: " << bestSnapshot;

            using namespace units::math;
            const auto maxHeading = robot.getMaximumTurn();
            heading = max(-maxHeading, min(maxHeading, heading));
            robot.steer(-heading);

            // Draw path on screen
            lineTesting.append(pose);
        }
    } while (!joystick.isPressed(HID::JButton::B) && display.isOpen());

    return EXIT_SUCCESS;
}
