/*
 * This program is for recording videos (stored on the drone) while the drone
 * spins around 360°, which can then be stitched together to create panoramic
 * images (see tools/image_stitcher).
 */

// BoB robotics includes
#include "common/logging.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/bebop/bebop.h"

// Third-party includes
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::literals;

int bob_main(int, char **)
{
    HID::Joystick joystick;
    Robots::Bebop drone;
    drone.addJoystick(joystick);

    joystick.addHandler([&drone](HID::JButton button, bool pressed) {
        auto animCompleted = [&drone](bool success) {
            drone.stopRecordingVideo();

            if (!success) {
                LOGW << "Animation prematurely stopped";
            }
        };

        if (pressed && button == HID::JButton::X) {
            if (drone.getAnimationState() == Robots::Bebop::AnimationState::Running) {
                drone.cancelCurrentAnimation();
            } else {
                try {
                    units::angle::degree_t angle;
                    units::angular_velocity::degrees_per_second_t speed;

                    // Note that this will only work outdoors and sometimes not even then!
                    std::tie(speed, angle) = drone.startHorizontalPanoramaAnimation(20_deg_per_s, animCompleted);
                    drone.startRecordingVideo();

                    LOGI << "Starting panorama:\n-Angle: " << angle
                         << "\nSpeed: " << speed;
                } catch (Robots::Bebop::AnimationError &) {
                    LOGE << "Could not start panorama animation; aborting";
                }
            }
            return true;
        } else {
            return false;
        }
    });

    /*
     * When drone starts landing, stop joystick running on main thread to
     * terminate program.
     */
    drone.setFlyingStateChangedHandler(
            [&joystick](auto state) {
                if (state == Robots::Bebop::FlyingState::Landing) {
                    joystick.stop();
                }
            });

    // Repeatedly poll joystick on main thread
    joystick.run();

    return EXIT_SUCCESS;
}
