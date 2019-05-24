// BoB robotics includes
#include "common/logging.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/bebop/bebop.h"

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <exception>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;
using namespace units::length;
using namespace units::angle;

int
bob_main(int, char **)
{
    // drone object
    Bebop drone;
    std::atomic<bool> stopFlag{ false };
    drone.setFlightEventHandler([&stopFlag](bool takeoff) {
        if (!takeoff) {
            stopFlag = true;
        }
    });

    // control drone with joystick
    HID::Joystick joystick;
    drone.addJoystick(joystick);
    joystick.runInBackground();

    std::string str;
    while (!stopFlag) {
        {
            LOGI << "Enter x: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            meter_t x(stod(str));

            LOGI << "Enter y: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            meter_t y(stod(str));

            LOGI << "Enter z: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            meter_t z(stod(str));

            LOGI << "Enter yawChange: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            degree_t yawChange(stod(str));

            LOGI << "Starting relative move...";
            drone.relativeMove(x, y, z, yawChange);
            while (drone.getRelativeMoveState() == Bebop::RelativeMoveState::Moving && !stopFlag) {
                std::this_thread::sleep_for(25ms);
            }
        }
        if (stopFlag) {
            break;
        }

        const auto moveState = drone.getRelativeMoveState();
        drone.resetRelativeMoveState();
        switch (moveState) {
        case Bebop::RelativeMoveState::ErrorBusy:
            LOGW << "Error: drone device is busy";
            continue;
        case Bebop::RelativeMoveState::ErrorInterrupted:
            LOGW << "Relative move operation was interrupted";
            break;
        case Bebop::RelativeMoveState::ErrorUnknown:
            LOGW << "Unknown error occurred while doing relative move";
            continue;
        case Bebop::RelativeMoveState::ErrorNotAvailable:
            LOGW << "Error: Relative move is not available on this device";
            continue;
        default:
            break;
        }

        Vector3<meter_t> positionChange;
        degree_t yawChange;
        std::tie(positionChange, yawChange) = drone.getRelativeMovePoseDifference();
        LOGI << "Drone moved by: (" << positionChange[0] << ", "
                  << positionChange[1] << ", " << positionChange[2] << ") and " << yawChange;
    }

    return EXIT_SUCCESS;
}
