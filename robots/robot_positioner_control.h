// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/fsm.h"
#include "common/pose.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/robot_positioner.h"
#include "robots/tank.h"
#include "vicon/udp.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

namespace BoBRobotics {
namespace Robots {
using namespace std::literals;
using namespace units::literals;

enum RobotPositionerControlState
{
    InvalidState,
    ControlWithJoystick,
    Homing
};

class RobotPositionerControl
  : FSM<RobotPositionerControlState>::StateHandler
{
    using Event = FSM<RobotPositionerControlState>::StateHandler::Event;
    using meter_t = units::length::meter_t;
    using radian_t = units::angle::radian_t;

public:
    RobotPositionerControl(Robots::Tank &tank,
                           Robots::RobotPositioner<Vicon::ObjectReference<>> &positioner,
                           HID::Joystick &joystick,
                           float joystickMaxSpeed = 1.f,
                           float positionerMaxSpeed = 0.5f)
      : m_Tank(tank)
      , m_Joystick(joystick)
      , m_Positioner(positioner)
      , m_StateMachine(this, InvalidState)
      , m_JoystickMaxSpeed(joystickMaxSpeed)
      , m_PositionerMaxSpeed(positionerMaxSpeed)
      , m_ReachedGoal(false)
      , m_StartHoming(false)
    {
        // Start controlling with joystick
        m_StateMachine.transition(ControlWithJoystick);
    }

    void setHomingStartedHandler(std::function<void()> handler)
    {
        m_HomingStartedHandler = handler;
    }

    void setHomingStoppedHandler(std::function<bool(bool)> handler)
    {
        m_HomingStoppedHandler = handler;
    }

    void run()
    {
        m_Catcher.trapSignals(); // Catch ctrl+c etc.

        std::cout << "Press Y to start homing" << std::endl;

        while (!m_Joystick.isPressed(HID::JButton::B)) {
            m_StateMachine.update();
            std::this_thread::sleep_for(20ms);
        }
    }

    virtual bool handleEvent(RobotPositionerControlState state, Event event) override
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
                std::cout << "Starting homing" << std::endl;
                std::cout << "Goal: " << getGoalPose() << std::endl;
                m_PrintTimer.start();
                m_Tank.setMaximumSpeedProportion(m_PositionerMaxSpeed);
                m_ReachedGoal = false;

                if (m_HomingStartedHandler) {
                    m_HomingStartedHandler();
                }
                break;
            case Event::Exit:
                std::cout << "Stopping homing" << std::endl;
                m_Positioner.reset();
                m_PrintTimer.reset();

                m_StartHoming = m_HomingStoppedHandler && m_HomingStoppedHandler(m_ReachedGoal);
                break;
            case Event::Update: {

                if (!m_Positioner.pollPositioner()) {
                    const auto &pose = m_Positioner.getPose();
                    std::cout << "Reached goal" << std::endl;
                    std::cout << "Final position: " << pose.x() << ", " << pose.y() << std::endl;
                    std::cout << "Goal: " << getGoalPose() << std::endl;
                    std::cout << "Distance to goal: "
                              << m_Positioner.distanceToGoal()
                              << " (" << circularDistance(getGoalPose().yaw(), pose.yaw()) << ")"
                              << std::endl;

                    m_ReachedGoal = true;
                    m_StateMachine.transition(ControlWithJoystick);
                    return true;
                }

                // Print status
                if (m_PrintTimer.elapsed() > 2s) {
                    m_PrintTimer.start();
                    std::cout << "Distance to goal: "
                              << m_Positioner.distanceToGoal()
                              << " (" << circularDistance(getGoalPose().yaw(), m_Positioner.getPose().yaw()) << ")"
                              << std::endl;
                }
            }
            }
        } else if (event == Event::Enter) { // Controlling with joystick
            if (m_StartHoming) {
                m_StateMachine.transition(Homing);
            } else {
                m_Tank.setMaximumSpeedProportion(m_JoystickMaxSpeed);
            }
        }

        return true;
    }

    void moveTo(const Pose2<meter_t, radian_t> &pose)
    {
        m_Positioner.moveTo(pose);
    }

    void startHoming()
    {
        m_StateMachine.transition(Homing);
    }

    void stopHoming()
    {
        m_StateMachine.transition(ControlWithJoystick);
    }

    const Pose2<meter_t, radian_t> &getGoalPose() const
    {
        return m_Positioner.getGoalPose();
    }

private:
    Robots::Tank &m_Tank;
    HID::Joystick &m_Joystick;
    Robots::RobotPositioner<Vicon::ObjectReference<>> &m_Positioner;
    FSM<RobotPositionerControlState> m_StateMachine;
    Stopwatch m_PrintTimer;
    BackgroundExceptionCatcher m_Catcher;
    std::function<void()> m_HomingStartedHandler;
    std::function<bool(bool)> m_HomingStoppedHandler;
    const float m_JoystickMaxSpeed, m_PositionerMaxSpeed;
    bool m_ReachedGoal;
    bool m_StartHoming;
}; // RobotPositionerControl
} // Robots
} // BoBRobotics
