// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/fsm.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "navigation/read_objects.h"
#include "net/client.h"
#include "robots/control/obstacle_circumnavigation.h"
#include "robots/control/robot_positioner.h"
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

public:
    PositionerExample()
      : m_Tank(m_Client)
      , m_Vicon(51001)
      , m_ViconObject(m_Vicon.getObjectReference(0))
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
      , m_CollisionDetector{ getRobotDimensions(m_Tank), Navigation::readObjects("objects.yaml"), 30_cm, 1_cm }
      , m_Circumnavigator{ m_Tank, m_ViconObject, m_CollisionDetector }
      , m_AvoidingPositioner(m_Positioner, m_Circumnavigator)
      , m_StateMachine(this, InvalidState)
    {
        // Goal is currently hard-coded
        std::cout << "Goal: " << Goal << std::endl;
        m_AvoidingPositioner.moveTo(Goal);

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
                m_PrintTimer.start();
                m_Tank.setMaximumSpeedProportion(PositionerMaxSpeed);
                break;
            case Event::Exit:
                std::cout << "Stopping homing" << std::endl;
                m_AvoidingPositioner.reset();
                m_PrintTimer.reset();
                break;
            case Event::Update: {
                try {
                    if (!m_AvoidingPositioner.pollPositioner()) {
                        const auto &finalPose = m_AvoidingPositioner.getPose();
                        std::cout << "Reached goal" << std::endl;
                        std::cout << "Final position: " << finalPose.x() << ", " << finalPose.y() << std::endl;
                        std::cout << "Goal: " << Goal << std::endl;
                        std::cout << "Distance to goal: "
                                  << Goal.distance2D(finalPose)
                                  << " (" << circularDistance(Goal.yaw(), finalPose.yaw()) << ")"
                                  << std::endl;

                        m_StateMachine.transition(ControlWithJoystick);
                        return true;
                    }
                } catch (Vicon::TimedOutError &) {
                    std::cerr << "Error: Could not get position from Vicon system\n"
                              << "Stopping trial" << std::endl;

                    m_StateMachine.transition(ControlWithJoystick);
                    return true;
                }

                // Print status
                if (m_PrintTimer.elapsed() > 500ms) {
                    m_PrintTimer.start();

                    const auto &pose = m_AvoidingPositioner.getPose();
                    std::cout << "Distance to goal: "
                              << Goal.distance2D(pose)
                              << " (" << circularDistance(Goal.yaw(), pose.yaw()) << ")"
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
    Vicon::ObjectReference<> m_ViconObject;
    HID::Joystick m_Joystick;
    Robots::RobotPositioner<Vicon::ObjectReference<>> m_Positioner;
    Robots::CollisionDetector m_CollisionDetector;
    Robots::ObstacleCircumnavigator<Vicon::ObjectReference<>> m_Circumnavigator;
    Robots::ObstacleAvoidingPositioner<Robots::RobotPositioner<Vicon::ObjectReference<>>, Vicon::ObjectReference<>> m_AvoidingPositioner;
    FSM<State> m_StateMachine;
    Stopwatch m_PrintTimer;
    BackgroundExceptionCatcher m_Catcher;

    static std::array<Vector2<meter_t>, 4> getRobotDimensions(Robots::Tank &tank)
    {
        using V = Vector2<meter_t>;
        const auto halfWidth = tank.getRobotWidth() / 2;

        // The x and y dimensions of the robot
        const std::array<V, 4> robotDimensions = {
            V{ -halfWidth, halfWidth },
            V{ halfWidth, halfWidth },
            V{ halfWidth, -halfWidth },
            V{ -halfWidth, -halfWidth }
        };
        return robotDimensions;
    }
};

int
bob_main(int, char **)
{


    PositionerExample example;
    example.run();

    return EXIT_SUCCESS;
}
