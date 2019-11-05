// BoB robotics includes
#include "common/logging.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/bebop/bebop.h"
#include "video/display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;
using namespace units::angle;
using namespace units::angular_velocity;

template<typename T>
using Limits = std::pair<T, T>;

void
printSpeedLimits(Bebop &drone)
{
    // max tilt
    using namespace units::angle;
    const degree_t maxTilt = drone.getMaximumTilt();
    const Limits<degree_t> tiltLimits = drone.getTiltLimits();
    LOGI << "Max tilt: " << maxTilt << " (limits: " << tiltLimits.first << ", "
              << tiltLimits.second << ")";

    // max yaw speed
    using namespace units::angular_velocity;
    const degrees_per_second_t maxYawSpeed = drone.getMaximumYawSpeed();
    const Limits<degrees_per_second_t> yawLimits = drone.getYawSpeedLimits();
    LOGI << "Max yaw speed: " << maxYawSpeed << " (limits: " << yawLimits.first << ", "
              << yawLimits.second << ")";

    // max vertical speed
    using namespace units::velocity;
    const meters_per_second_t maxVertSpeed = drone.getMaximumVerticalSpeed();
    const Limits<meters_per_second_t> vertSpeedLimits = drone.getVerticalSpeedLimits();
    LOGI << "Max vertical speed: " << maxVertSpeed << " (limits: " << vertSpeedLimits.first << ", "
              << vertSpeedLimits.second << ")";
}

int
bob_main(int, char **)
{
    /*
     * Connects to the drone.
     *
     * NB: Any or all of these parameters can be omitted to use the defaults,
     *     which is probably almost always what you want. Side note: don't set
     *     these values to their maximum if you want to be able to control the
     *     drone.
     */
    Bebop drone(/*maxYawSpeed=*/Bebop::DefaultMaximumYawSpeed,
                /*maxVerticalSpeed=*/Bebop::DefaultMaximumVerticalSpeed,
                /*maxTilt=*/Bebop::DefaultMaximumTilt);

    LOGI << "Battery is at: " << drone.getBatteryLevel() * 100.f << "%";

    // print maximum speed parameters
    printSpeedLimits(drone);

    // control drone with joystick
    HID::Joystick joystick(/*deadZone=*/0.25);
    drone.addJoystick(joystick);

    // display the drone's video stream on screen
    Video::Display display(drone.getVideoStream(), true);
    bool animationRunning = false;
    do {
        bool joyUpdated = joystick.update();
        if (joyUpdated) {
            if (joystick.isPressed(HID::JButton::X)) {
                if (!drone.isVideoRecording()) {
                    LOGI << "Starting video recording";
                    drone.startRecordingVideo();
                } else {
                    LOGI << "Stopping video recording";
                    drone.stopRecordingVideo();
                }
            }
            if (joystick.isPressed(HID::JButton::RB)) {
                if (animationRunning) {
                    drone.cancelCurrentAnimation();
                    animationRunning = false;
                } else {
                    try {
                        degrees_per_second_t speed;
                        degree_t angle;
                        std::tie(speed, angle) = drone.startHorizontalPanoramaAnimation(20_deg_per_s);
                        LOGI << "Running animation: \n- Rotation: " << angle
                             << "\n- Speed: " << speed;
                        animationRunning = true;
                    } catch (std::runtime_error &) {
                        LOGW << "Animation failed to start";
                    }
                }
            }
        }

        bool dispUpdated = display.update();
        if (!joyUpdated && !dispUpdated) {
            std::this_thread::sleep_for(25ms);
        }
    } while (display.isOpen());

    return EXIT_SUCCESS;
}
