// BoB robotics includes
#include "robots/control/robot_positioner.h"
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/fsm.h"
#include "common/logging.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/robot_type.h"
#include "vicon/udp.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::length;
using namespace units::literals;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::velocity;
using namespace std::literals;

enum State
{
    InvalidState,
    ControlWithJoystick,
    Homing
};

// Positioner parameters
constexpr meter_t StoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
constexpr radian_t AllowedHeadingError = 5_deg; // the amount of error allowed in the final heading
constexpr double K1 = 0.2;                      // curveness of the path to the goal
constexpr double K2 = 5;                        // speed of turning on the curves
constexpr double Alpha = 1.03;                  // causes more sharply peaked curves
constexpr double Beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases
constexpr Pose2<millimeter_t, degree_t> Goal{ 0_mm, 0_mm, 15_deg };

// Speed parameters
constexpr float JoystickMaxSpeed = 1.f;
constexpr float PositionerMaxSpeed = 0.5f;
constexpr float PositionerMinSpeed = 0.2f;
constexpr meter_t StartSlowingDownAt = 40_cm;

class PositionerExample
  : FSM<State>::StateHandler
{
private:
    using Event = FSM<State>::StateHandler::Event;

public:
    PositionerExample()
      : m_Vicon(51001)
      , m_ViconObject(m_Vicon.getObjectReference())
      , m_Positioner(m_Tank,
                     m_ViconObject,
                     StoppingDistance,
                     AllowedHeadingError,
                     K1,
                     K2,
                     Alpha,
                     Beta,
                     StartSlowingDownAt,
                     PositionerMinSpeed,
                     PositionerMaxSpeed)
      , m_StateMachine(this, InvalidState)
    {
        // Goal is currently hard-coded
        LOGI << "Goal: " << Goal;
        m_Positioner.moveTo(Goal);

        // Start controlling with joystick
        m_StateMachine.transition(ControlWithJoystick);
    }

    void run()
    {
        m_Catcher.trapSignals(); // Catch ctrl+c etc.

        LOGI << "Press Y to start homing";

        while (!m_Joystick.isPressed(HID::JButton::B)) {
            m_StateMachine.update();

            std::this_thread::sleep_for(20ms);
        }
    }

    virtual bool handleEvent(State state, Event event) override
    {
        if (state == InvalidState) {
            return true;
        }

        if (event == Event::Exit) {
            m_Tank.stopMoving();
        } else if (event == Event::Update) {
            // Check for background exceptions
            m_Catcher.check();

            // Poll for joystick events
            if (m_Joystick.update()) {
                const auto state = m_StateMachine.getCurrentState();

                // Toggle whether the positioner is "on" with Y button
                if (m_Joystick.isPressed(HID::JButton::Y)) {
                    if (state == ControlWithJoystick) {
                        m_StateMachine.transition(Homing);
                    } else if (state == Homing) {
                        m_StateMachine.transition(ControlWithJoystick);
                    }
                } else if (state == ControlWithJoystick) {
                    m_Tank.drive(m_Joystick);
                }
            }
        }

        if (state == Homing) {
            switch (event) {
            case Event::Enter:
                LOGI << "Starting homing";
                m_PrintTimer.start();
                m_Tank.setMaximumSpeedProportion(PositionerMaxSpeed);
                break;
            case Event::Exit:
                LOGI << "Stopping homing";
                m_Positioner.reset();
                m_PrintTimer.reset();
                break;
            case Event::Update: {
                try {
                    if (!m_Positioner.pollPositioner()) {
                        const auto &finalPose = m_Positioner.getPose();
                        LOGI << "Reached goal";
                        LOGI << "Final position: " << finalPose.x() << ", " << finalPose.y();
                        LOGI << "Goal: " << Goal;
                        LOGI << "Distance to goal: "
                             << Goal.distance2D(finalPose)
                             << " (" << circularDistance(Goal.yaw(), finalPose.yaw()) << ")";

                        m_StateMachine.transition(ControlWithJoystick);
                        return true;
                    }
                } catch (Vicon::TimedOutError &) {
                    LOGE << "Could not get position from Vicon system\n"
                         << "Stopping trial";

                    m_StateMachine.transition(ControlWithJoystick);
                    return true;
                }

                // Print status
                if (m_PrintTimer.elapsed() > 500ms) {
                    m_PrintTimer.start();

                    const auto &pose = m_Positioner.getPose();
                    LOGI << "Distance to goal: "
                         << Goal.distance2D(pose)
                         << " (" << circularDistance(Goal.yaw(), pose.yaw()) << ")";
                }
            }
            }
        } else if (event == Event::Enter) { // Controlling with joystick
            m_Tank.setMaximumSpeedProportion(JoystickMaxSpeed);
        }

        return true;
    }

private:
    Robots::ROBOT_TYPE m_Tank;
    Vicon::UDPClient<> m_Vicon;
    Vicon::ObjectReference<> m_ViconObject;
    HID::Joystick m_Joystick;
    Robots::RobotPositioner<Vicon::ObjectReference<>> m_Positioner;
    FSM<State> m_StateMachine;
    Stopwatch m_PrintTimer;
    BackgroundExceptionCatcher m_Catcher;
};

int
bob_main(int, char **)
{
    PositionerExample example;
    example.run();

    return EXIT_SUCCESS;
}
