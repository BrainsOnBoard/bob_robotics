// BoB robotics includes
#include "common/logging.h"
#include "common/main.h"
#include "common/path.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "os/keycodes.h"
#include "robots/bebop/bebop.h"
#include "vicon/udp.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <chrono>
#include <memory>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::time;

class RouteDatabaseCreator {
public:
    RouteDatabaseCreator(const filesystem::path &dir)
      : m_ImageDatabase{ Path::getNewPath(dir) }
      , m_Recorder{ m_ImageDatabase.getRouteRecorder() }
    {
        LOGI << "Recording started";
    }

    ~RouteDatabaseCreator()
    {
        LOGI << "Recording stopped";
    }

    void record(const Pose3<meter_t, radian_t> &pose, const cv::Mat &frame)
    {
        m_Recorder.record(pose.position(), pose.yaw(), frame);
    }

private:
    Navigation::ImageDatabase m_ImageDatabase;
    Navigation::ImageDatabase::RouteRecorder m_Recorder;
};

int bob_main(int, char **argv)
{
    constexpr auto updateRate = 200ms;
    HID::Joystick joystick;
    Robots::Bebop drone;
    auto &camera = drone.getVideoStream();
    drone.addJoystick(joystick);
    Vicon::UDPClient<> vicon{ 51001 };
    const auto myPath = filesystem::path{ argv[0] }.parent_path();
    std::unique_ptr<RouteDatabaseCreator> creator;
    Stopwatch recordTimer;
    cv::Mat frame;

    LOGI << "Drone battery: " << drone.getBatteryLevel() * 100.f;

    do {
        // Toggle whether recording images
        if (joystick.update() && joystick.isPressed(HID::JButton::Y)) {
            if (!creator) {
                // Start recording
                creator = std::make_unique<RouteDatabaseCreator>(myPath);
                recordTimer.start();
            } else {
                // Stop recording, write to disk
                creator.reset();
            }
        }

        // Read from video stream
        camera.readFrameSync(frame);

        // If we're recording, then save image with pose
        if (creator && recordTimer.elapsed() >= updateRate) {
            recordTimer.start(); // Restart stopwatch
            const auto objectData = vicon.getObjectData();
            creator->record(objectData.getPose(), frame);

            // Check for out of date position
            const auto duration = objectData.timeSinceReceived();
            if (duration > 1s) {
                LOGW << "Using Vicon position from "
                     << static_cast<second_t>(duration)
                     << " ago";
            }
        }

        // Show video stream on screen
        cv::imshow("BoB robotics", frame);
    } while (!joystick.isPressed(HID::JButton::B) &&
             (cv::waitKeyEx(1) & OS::KeyMask) != OS::KeyCodes::Escape);

    return EXIT_SUCCESS;
}
