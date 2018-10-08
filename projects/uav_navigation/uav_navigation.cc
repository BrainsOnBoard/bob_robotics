// BoB robotics includes
#include "common/assert.h"
#include "hid/joystick.h"
#include "libbebop/bebop.h"
#include "navigation/perfect_memory.h"
#include "os/keycodes.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::literals;
using namespace units::time;

using TimeType = std::chrono::time_point<std::chrono::high_resolution_clock>;
namespace plt = matplotlibcpp;

class UAVNavigation
{
    using DataVector = std::vector<std::pair<TimeType, float>>;

public:
    UAVNavigation(const degree_t halfScanWidth, const degrees_per_second_t yawSpeed)
      : m_PerfectMemory(m_Drone.getVideoStream().getOutputSize())
      , m_HalfScanWidth(halfScanWidth)
      , m_YawSpeed(yawSpeed)
      , m_ProportionYawSpeed(yawSpeed / m_Drone.getMaximumYawSpeed())
    {
        BOB_ASSERT(halfScanWidth > 0_deg);
        BOB_ASSERT(yawSpeed > 0_deg_per_s);

        std::cout << "Connected to drone" << std::endl;

        m_Joystick.addHandler([this](HID::JButton button, bool pressed) {
            return onButtonEvent(button, pressed);
        });
        m_Joystick.addHandler([this](HID::JAxis axis, float) {
            if (axis != HID::JAxis::RightStickHorizontal && axis != HID::JAxis::RightStickVertical && m_NavigationState != NotNavigating) {
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
        DataVector acwData, cwData;
        bool plotShown = false;
        do {
            const bool joystickUpdate = m_Joystick.update();
            const bool cameraUpdate = camera.readFrame(m_Frame);
            if (plotShown) {
                plt::pause(0.025);
            } else if (!cameraUpdate && !joystickUpdate) {
                std::this_thread::sleep_for(25ms);
                continue;
            }
            if (cameraUpdate) {
                cv::imshow("Camera stream", m_Frame);
            }
            if (m_NavigationState == NotNavigating) {
                continue;
            }

            const auto currentTime = now();
            if ((currentTime - m_StartTime) >= halfScanDuration) {
                switch (m_NavigationState) {
                case ScanningAntiClockwise:
                    startReturnToCentre(currentTime, 1.f);
                    m_NavigationState = ClockwiseToCentre;
                    break;
                case ClockwiseToCentre:
                    startScanningClockwise(currentTime);
                    break;
                case ScanningClockwise:
                    startReturnToCentre(currentTime, -1.f);
                    m_NavigationState = AntiClockwiseToCentre;
                    break;
                case AntiClockwiseToCentre:
                    stopNavigating();
                    plotNavigationResults(acwData, cwData);
                    plotShown = true;
                default:
                    break;
                }
            } else if (m_NavigationState == ScanningAntiClockwise || m_NavigationState == ScanningClockwise) {
                cv::cvtColor(m_Frame, m_FrameGreyscale, cv::COLOR_RGB2GRAY);
                const float match = m_PerfectMemory.test(m_FrameGreyscale);
                auto datum = std::make_pair<>(currentTime, match);
                if (m_NavigationState == ScanningAntiClockwise) {
                    acwData.emplace_back(std::move(datum));
                } else {
                    cwData.emplace_back(std::move(datum));
                }
            }
        } while (m_Drone.getState() == Robots::Bebop::State::Running && (cv::waitKeyEx(1) & OS::KeyMask) != OS::KeyCodes::Escape);
    }

private:
    HID::Joystick m_Joystick;
    Robots::Bebop m_Drone;
    Navigation::PerfectMemory<> m_PerfectMemory;
    const degree_t m_HalfScanWidth;
    const degrees_per_second_t m_YawSpeed;
    const float m_ProportionYawSpeed;
    enum State
    {
        NotNavigating = 0,
        ScanningAntiClockwise,
        ClockwiseToCentre,
        ScanningClockwise,
        AntiClockwiseToCentre
    } m_NavigationState = NotNavigating;
    cv::Mat m_Frame, m_FrameGreyscale;
    TimeType m_StartTime, m_AntiClockwiseStartTime, m_ClockwiseStartTime;

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

    void stopNavigating()
    {
        m_Drone.stopMoving();
        m_NavigationState = NotNavigating;
        std::cout << "Stopping navigation" << std::endl;
    }

    void startNavigating()
    {
        m_StartTime = m_AntiClockwiseStartTime = now();
        m_Drone.turnOnTheSpot(-m_ProportionYawSpeed);
        m_NavigationState = ScanningAntiClockwise;
        std::cout << "Scanning anticlockwise" << std::endl;
    }

    void startReturnToCentre(TimeType currentTime, float yawDirection)
    {
        m_StartTime = currentTime;
        m_Drone.turnOnTheSpot(yawDirection * m_ProportionYawSpeed);
        std::cout << "Returning to centre" << std::endl;
    }

    void startScanningClockwise(TimeType currentTime)
    {
        m_StartTime = m_ClockwiseStartTime = currentTime;
        m_NavigationState = ScanningClockwise;
        std::cout << "Scanning clockwise" << std::endl;
    }

    void plotNavigationResults(DataVector &acwData, DataVector &cwData)
    {
        std::vector<float> headings;
        headings.reserve(acwData.size() + cwData.size());
        std::transform(acwData.crbegin(), acwData.crend(), std::back_inserter(headings),
            [this](const auto &datum) {
                return -timeToHeading(datum.first, m_AntiClockwiseStartTime);
            });
        std::transform(cwData.cbegin(), cwData.cend(), std::back_inserter(headings),
            [this](const auto &datum) {
                return timeToHeading(datum.first, m_ClockwiseStartTime);
            });

        const auto normalise = [](const auto &datum) {
            return datum.second / 255.f;
        };
        std::vector<float> scores;
        scores.reserve(headings.size());
        std::transform(acwData.crbegin(), acwData.crend(), std::back_inserter(scores), normalise);
        std::transform(cwData.cbegin(), cwData.cend(), std::back_inserter(scores), normalise);

        const auto bestIter = std::min_element(scores.cbegin(), scores.cend());
        const size_t bestIndex = std::distance(scores.cbegin(), bestIter);
        std::cout << "Best match was " << *bestIter
                  << " at approx " << headings[bestIndex] << std::endl;

        plt::plot(headings, scores);

        acwData.clear();
        cwData.clear();
    }

    float timeToHeading(TimeType time, TimeType startTime)
    {
        const second_t timeUnits = startTime - time;
        const degree_t heading = m_YawSpeed * timeUnits;
        return heading.value();
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
        UAVNavigation nav(45_deg, 20_deg_per_s);
        nav.mainLoop();
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}