// BoB robotics includes
#include "robots/robot_positioner.h"
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/fsm.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <chrono>
#include <iostream>
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

class PositionerExample
  : FSM<State>::StateHandler
{
private:
    using Event = FSM<State>::StateHandler::Event;

    // Positioner parameters
    static constexpr meter_t StoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
    static constexpr radian_t AllowedHeadingError = 5_deg; // the amount of error allowed in the final heading
    static constexpr double K1 = 0.2;                      // curveness of the path to the goal
    static constexpr double K2 = 5;                        // speed of turning on the curves
    static constexpr double Alpha = 1.03;                  // causes more sharply peaked curves
    static constexpr double Beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases
    const Pose2<millimeter_t, degree_t> Goal{ 0_mm, 0_mm, 15_deg };

    // Speed parameters
    static constexpr float JoystickMaxSpeed = 1.f;
    static constexpr float PositionerMaxSpeed = 0.5f;
    static constexpr float PositionerMinSpeed = 0.2f;
    static constexpr meter_t StartSlowingDownAt = 40_cm;

public:
    PositionerExample()
      : m_Tank(m_Client)
      , m_Vicon(51001)
      , m_Positioner(StoppingDistance,
                     AllowedHeadingError,
                     K1,
                     K2,
                     Alpha,
                     Beta)
      , m_StateMachine(this, InvalidState)
    {
        // Goal is currently hard-coded
        std::cout << "Goal: " << Goal << std::endl;
        m_Positioner.setGoalPose(Goal);

        // Drive robot with joystick
        m_Tank.addJoystick(m_Joystick);

        // Toggle homing with Y button
        m_Joystick.addHandler([this](HID::JButton button, bool pressed) {
            if (pressed && button == HID::JButton::Y) {
                const auto state = m_StateMachine.getCurrentState();
                if (state == ControlWithJoystick) {
                    m_StateMachine.transition(Homing);
                } else if (state == Homing) {
                    m_StateMachine.transition(ControlWithJoystick);
                }
                return true;
            } else {
                return false;
            }
        });

        // Ignore thumbsticks while robot is homing
        m_Joystick.addHandler([this](HID::JAxis, float) {
            if (m_StateMachine.getCurrentState() == Homing) {
                std::cout << "Ignoring joystick while homing" << std::endl;
                return true;
            } else {
                return false;
            }
        });

        // Wait for Vicon system to spot our robot
        while (m_Vicon.getNumObjects() == 0) {
            std::this_thread::sleep_for(1s);
            std::cout << "Waiting for object" << std::endl;
        }

        // Start controlling with joystick
        m_StateMachine.transition(ControlWithJoystick);
    }

    void run()
    {
        m_Catcher.trapSignals(); // Catch ctrl+c etc.

        // Listen in background
        m_Client.runInBackground();

        std::cout << "Press Y to start homing" << std::endl;

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
            m_Joystick.update();
        }

        if (state == Homing) {
            switch (event) {
            case Event::Enter:
                std::cout << "Starting homing" << std::endl;
                m_PrintTimer.start();
                m_Tank.setMaximumSpeedProportion(PositionerMaxSpeed);
                break;
            case Event::Exit:
                std::cout << "Stopping homing" << std::endl;
                m_Positioner.reset();
                m_PrintTimer.reset();
                break;
            case Event::Update: {
                const auto objectData = m_Vicon.getObjectData(0);
                const auto position = objectData.getPosition();
                const auto attitude = objectData.getAttitude();
                if (objectData.timeSinceReceived() > 10s) {
                    std::cerr << "Error: Could not get position from Vicon system\n"
                              << "Stopping trial" << std::endl;

                    m_StateMachine.transition(ControlWithJoystick);
                    return true;
                }

                if (m_Positioner.reachedGoal()) {
                    std::cout << "Reached goal" << std::endl;
                    std::cout << "Final position: " << position.x() << ", " << position.y() << std::endl;
                    std::cout << "Goal: " << Goal << std::endl;
                    std::cout << "Distance to goal: "
                              << Goal.distance2D(position)
                              << " (" << circularDistance(Goal.yaw(), attitude[0]) << ")"
                              << std::endl;

                    m_StateMachine.transition(ControlWithJoystick);
                    return true;
                }

                const auto distance = Goal.distance2D(position);
                if (distance < StartSlowingDownAt) {
                    const auto speedRange = PositionerMaxSpeed - PositionerMinSpeed;
                    const auto speedProp = speedRange * distance / StartSlowingDownAt;
                    m_Tank.setMaximumSpeedProportion(PositionerMinSpeed + speedProp);
                }

                m_Positioner.updateMotors(m_Tank, objectData.getPose());

                // Print status
                if (m_PrintTimer.elapsed() > 500ms) {
                    m_PrintTimer.start();
                    std::cout << "Distance to goal: "
                                << distance
                                << " (" << circularDistance(Goal.yaw(), attitude[0]) << ")"
                                << std::endl;
                }
            }
            }
        } else if (event == Event::Enter) { // Controlling with joystick
            m_Tank.setMaximumSpeedProportion(JoystickMaxSpeed);
        }

        return true;
    }

private:
    Net::Client m_Client;
    Robots::TankNetSink m_Tank;
    Vicon::UDPClient<> m_Vicon;
    HID::Joystick m_Joystick;
    Robots::RobotPositioner m_Positioner;
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