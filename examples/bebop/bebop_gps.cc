// BoB robotics includes
#include "common/logging.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/bebop/bebop.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;
using namespace units::angle;

int
bob_main(int, char **)
{
    HID::Joystick joystick;
    Bebop drone;
    LOGI << "Battery is at: " << drone.getBatteryLevel() * 100.f << "%";

    // Store GPS position on take-off so we can home to it, quit when landing
    MapCoordinate::GPSCoordinate takeOffCoords{ unan<degree_t>(), unan<degree_t>() };
    drone.setFlyingStateChangedHandler([&](Bebop::FlyingState state) {
        Bebop::GPSData data;
        switch (state) {
        case Bebop::FlyingState::Flying:
            if (drone.getGPSData(data)) {
                takeOffCoords = data.coordinate;
                LOGI << "Home GPS position: " << takeOffCoords.lat << " " << takeOffCoords.lon;
            } else {
                LOGW << "Could not get GPS position; will not home :-(";
            }
            break;
        case Bebop::FlyingState::Landed:
        case Bebop::FlyingState::Emergency:
            // Terminate program
            joystick.stop();
        default:
            break;
        }
    });

    // control drone with joystick
    drone.addJoystick(joystick);

    // Initialise homing when X button pressed
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::X) {
            if (drone.getMoveToState() == Bebop::MoveToState::Running) {
                drone.cancelMoveTo();
            } else {
                Bebop::GPSData data;
                if (drone.getGPSData(data)) {
                    takeOffCoords.height = data.coordinate.height;
                    LOGI << "Initialising homing";
                    drone.moveTo(takeOffCoords, [&](bool success) {
                        if (success) {
                            LOGI << "Homed successfully!";
                        } else {
                            LOGE << "Homing terminated prematurely";
                        }
                    });
                } else {
                    LOGW << "Could not get GPS position; homing aborted";
                }
            }
            return true;
        } else {
            return false;
        }
    });

    // Poll on main thread
    joystick.run();
    return EXIT_SUCCESS;
}
