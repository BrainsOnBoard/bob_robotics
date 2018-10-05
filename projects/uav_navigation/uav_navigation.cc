// BoB robotics includes
#include "hid/joystick.h"
#include "libbebop/bebop.h"
#include "navigation/perfect_memory.h"
#include "os/keycodes.h"

// Third-party includes
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::literals;
using namespace units::time;

using TimeType = std::chrono::time_point<std::chrono::high_resolution_clock>;

class UAVNavigation
{
public:
    UAVNavigation(degree_t halfScanWidth, degrees_per_second_t yawSpeed)
      : m_PerfectMemory(m_Drone.getVideoStream().getOutputSize())
      , m_HalfScanWidth(halfScanWidth)
      , m_YawSpeed(yawSpeed)
      , m_ProportionYawSpeed(yawSpeed / m_Drone.getMaximumYawSpeed())
    {
        std::cout << "Connected to drone" << std::endl;

        // Quit if drone lands
        m_Drone.setFlightEventHandler([this](bool takeoff) {
            if (!takeoff) {
                m_StopFlag = true;
            }
        });

        m_Joystick.addHandler([this](HID::JButton button, bool pressed) {
            return onButtonEvent(button, pressed);
        });
        m_Joystick.addHandler([this](HID::JAxis, float)
        {
            if (m_NavigationState != NotNavigating) {
                stopNavigating();
            }
            return false;
        });

        // Control drone with joystick
        m_Drone.addJoystick(m_Joystick);
    }

    void mainLoop()
    {
        const nanosecond_t halfScanDurationUnits = m_HalfScanWidth / m_YawSpeed;
        const std::chrono::nanoseconds halfScanDuration((int64_t) halfScanDurationUnits.value());

        std::cout << "Scanning for " << static_cast<millisecond_t>(halfScanDurationUnits) << std::endl;

        auto &camera = m_Drone.getVideoStream();
        TimeType bestHeadingTime;
        float bestMatch = std::numeric_limits<float>::infinity();
        do {
            const bool joystickUpdate = m_Joystick.update();
            const bool cameraUpdate = camera.readFrame(m_Frame);
            if (!cameraUpdate && !joystickUpdate) {
                std::this_thread::sleep_for(25ms);
                continue;
            }
            if (cameraUpdate) {
                cv::imshow("Camera stream", m_Frame);
            }
            if (m_NavigationState == TurningAntiClockwise) {
                const auto currentTime = now();
                if ((currentTime - m_ScanStartTime) >= halfScanDuration) {
                    stopNavigating();
                    const second_t bestHeadingTimeUnits = bestHeadingTime - m_ScanStartTime;
                    const degree_t bestHeading = -m_YawSpeed * bestHeadingTimeUnits;
                    std::cout << "Best match was " << bestMatch << " at approx " << bestHeading << std::endl;
                } else {
                    cv::cvtColor(m_Frame, m_FrameGreyscale, cv::COLOR_RGB2GRAY);
                    const float match = m_PerfectMemory.test(m_FrameGreyscale);
                    if (match < bestMatch) {
                        bestMatch = match;
                        bestHeadingTime = currentTime;
                    }
                }
            }
        } while (!m_StopFlag && (cv::waitKeyEx(1) & OS::KeyMask) != OS::KeyCodes::Escape);
    }

private:
    HID::Joystick m_Joystick;
    Robots::Bebop m_Drone;
    Navigation::PerfectMemory<> m_PerfectMemory;
    degree_t m_HalfScanWidth;
    degrees_per_second_t m_YawSpeed;
    float m_ProportionYawSpeed;
    enum State
    {
        NotNavigating = 0,
        TurningAntiClockwise,
        TurningClockwise
    } m_NavigationState = NotNavigating;
    cv::Mat m_Frame, m_FrameGreyscale;
    bool m_StopFlag = false;
    TimeType m_ScanStartTime;

    bool onButtonEvent(HID::JButton button, bool pressed)
    {
        if (!pressed) {
            return false;
        }

        if (m_NavigationState != NotNavigating) {
            stopNavigating();
            return false;
        } else {
            switch (button) {
            case HID::JButton::X:
                if (!m_Frame.empty()) {
                    cv::cvtColor(m_Frame, m_FrameGreyscale, cv::COLOR_RGB2GRAY);
                    m_PerfectMemory.train(m_FrameGreyscale);
                    std::cout << "Snapshot added (n=" << m_PerfectMemory.getNumSnapshots()
                              << ")" << std::endl;
                }
                return true;
            case HID::JButton::Y:
                m_PerfectMemory.clearMemory();
                std::cout << "Memory cleared" << std::endl;
                return true;
            case HID::JButton::Start:
                startNavigating();
                return true;
            default:
                // Button press not handled
                return false;
            }
        }
    }

    void startNavigating()
    {
        m_NavigationState = TurningAntiClockwise;
        m_ScanStartTime = now();
        m_Drone.turnOnTheSpot(-m_ProportionYawSpeed);
        std::cout << "Starting navigation" << std::endl;
    }

    void stopNavigating()
    {
        m_Drone.stopMoving();
        m_NavigationState = NotNavigating;
        std::cout << "Stopping navigation" << std::endl;
    }

    static TimeType now()
    {
        return std::chrono::high_resolution_clock::now();
    }
};

int
main()
{
    try {
        std::cout << "Connecting to drone..." << std::endl;
        UAVNavigation nav(45_deg, 20_deg_per_s);
        nav.mainLoop();
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}