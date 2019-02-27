// BoB robotics includes
#include "hid/joystick.h"
#include "libbebop/bebop.h"

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;

int
main()
{
    try {
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
                std::cout << "Enter dx: ";
                std::getline(std::cin, str);
                if (str.empty()) {
                    return 0;
                }
                meter_t dx(stod(str));

                std::cout << "Enter dy: ";
                std::getline(std::cin, str);
                if (str.empty()) {
                    return 0;
                }
                meter_t dy(stod(str));

                std::cout << "Enter dz: ";
                std::getline(std::cin, str);
                if (str.empty()) {
                    return 0;
                }
                meter_t dz(stod(str));

                std::cout << "Enter dpsi: ";
                std::getline(std::cin, str);
                if (str.empty()) {
                    return 0;
                }
                degree_t dpsi(stod(str));

                std::cout << "Starting relative move..." << std::endl;
                drone.relativeMove(dx, dy, dz, dpsi);
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
                std::cerr << "Error: drone device is busy" << std::endl;
                continue;
            case Bebop::RelativeMoveState::ErrorInterrupted:
                std::cerr << "Relative move operation was interrupted" << std::endl;
                break;
            case Bebop::RelativeMoveState::ErrorUnknown:
                std::cerr << "Unknown error occurred while doing relative move" << std::endl;
                continue;
            case Bebop::RelativeMoveState::ErrorNotAvailable:
                std::cerr << "Error: Relative move is not available on this device" << std::endl;
                continue;
            default:
                break;
            }

            Vector3<meter_t> dposition;
            degree_t dpsi;
            std::tie(dposition, dpsi) = drone.getRelativeMovePoseDifference();
            std::cout << "Drone moved by: (" << dposition[0] << ", "
                      << dposition[1] << ", " << dposition[2] << ") and " << dpsi << std::endl;
        }
    } catch (std::exception &e) {
        /*
         * We catch all exceptions, because otherwise drone's destructor won't
         * be called and the drone might be left hovering (or flying towards a
         * wall).
         */
        std::cerr << "ERROR: Uncaught exception: " << e.what() << std::endl;
    }
}
