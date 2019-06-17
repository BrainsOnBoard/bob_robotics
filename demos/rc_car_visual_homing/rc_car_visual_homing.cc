// BoB robotics includes
#include "antworld/camera.h"
#include "common/logging.h"
#include "common/macros.h"
#include "common/main.h"
#include "common/map_coordinate.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/simulated_ackermann_car.h"
#include "video/randominput.h"
#include "viz/sfml_world/arena_object.h"
#include "viz/sfml_world/sfml_world.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>

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

filesystem::path
getNewFilepath(const filesystem::path folderPath, const std::string &fileExtension)
{
    makeDirectory(folderPath);

    // Put a timestamp in the filename
    std::stringstream ss;
    const auto timer = time(nullptr);
    const auto currentTime = localtime(&timer);
    ss << std::setfill('0')
       << std::setw(2) << currentTime->tm_mday
       << std::setw(2) << currentTime->tm_mon
       << std::setw(2) << currentTime->tm_year - 100
       << "_"
       << std::setw(2) << currentTime->tm_hour
       << std::setw(2) << currentTime->tm_min
       << std::setw(2) << currentTime->tm_sec
       << "_";
    const auto basename = (folderPath / ss.str()).str();

    // Append a number in case we get name collisions
    ss.str(std::string{}); // clear stringstream
    filesystem::path path;
    for (int i = 1; ; i++) {
        ss << i << fileExtension;
        path = basename + ss.str();
        if (!path.exists()) {
            break;
        }
        ss.str(std::string{}); // clear stringstream
    }
    return path;
}

int
bob_main(int, char **argv)
{
    const auto programPath = filesystem::path{ argv[0] }.parent_path();
    const auto videoFilepath = getNewFilepath(programPath / "videos", ".avi");
    const auto dataFilepath = getNewFilepath(programPath / "data", ".yaml");
    std::atomic_bool stopFlag{ false };

    const cv::Size RenderSize{ 720, 150 };
    const meter_t AntHeight = 1_cm;

    HID::Joystick joystick;

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
    Robots::SimulatedAckermannCar<meter_t> robot{ 1_mph, 20_cm, 10_cm };
    robot.addJoystick(joystick);

    // Visualisation of robot position
    auto carAgent = display.createCarAgent(robot.getDistanceBetweenAxis());

    // Log data to YAML file
    LOGI << "Saving data to " << dataFilepath;
    cv::FileStorage fs{ dataFilepath.str(), cv::FileStorage::WRITE };
    BOB_ASSERT(fs.isOpened());
    fs << "data" << "{" << "video_filepath" << videoFilepath.str();

    cv::Mat fr;
    Stopwatch stopwatch;
    stopwatch.start();
    fs << "coords"
       << "["; // YAML array
    do {
        joystick.update();
        const auto pose = robot.getPose();

        // Render in visualisation
        carAgent.setPose(pose);
        display.update(objects, carAgent);

        // Pretend to do something with camera
        antCamera.setPose(pose);
        antCamera.update();

        // Log position of robot
        const std::chrono::duration<double, std::milli>
                time = stopwatch.elapsed();
        fs << "{"
           << "time" << time.count()
           << "pose" << pose
           << "}";
    } while (!joystick.isPressed(HID::JButton::B) && display.isOpen());
    fs << "]"
       << "}";
    fs.release();

    // Stop writing video
    stopFlag = true;

    return EXIT_SUCCESS;
}
