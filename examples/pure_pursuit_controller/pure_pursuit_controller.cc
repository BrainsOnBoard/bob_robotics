/*
    Small example program to demonstrate the Pure Pursuit Controller. The controller
    takes a list of coordinates as waypoint and calculates a desired turning angle
    to steer the car for a smooth path tracking.
    To create a path, press the left mouse button to place a waypoint to the screen
    making a path. Then, pressing the space button will start the controller algorithm
    to make the robot follow the path

    key commands:
        [    ARROW UP     ] = apply max speed to car
        [   ARROW DOWN    ] = stop car
        [   ARROW LEFT    ] = turn steering wheel by 30 degrees left
        [   ARROW RIGHT   ] = turn steering wheel by 30 degrees right
        [      SPACE      ] = toggle controller algorithm on/off
        [LEFT MOUSE BUTTON] = places a waypoint on the screen adding it to the path
*/

// BoB robotics includes
#include "common/fsm.h"
#include "robots/control/pure_pursuit_controller.h"
#include "robots/ackermann/simulated_ackermann.h"
#include "viz/sfml/robot_control.h"
#include "viz/sfml/world.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace units::literals;
using namespace units::length;
using namespace units::angle;
using namespace units::velocity;
using namespace units::math;

using namespace BoBRobotics;
using namespace BoBRobotics::Robots::Ackermann;
using namespace BoBRobotics::Viz::SFML;

constexpr millimeter_t LookAheadDistance = 1_m;   // lookahead distance
constexpr meters_per_second_t MaxSpeed = 1.4_mps; // car's max speed
constexpr degree_t MaxTurn = 30_deg;              // car's maximum turning angle
constexpr millimeter_t StoppingDist = 1_cm;       // car's stopping distance

enum State
{
    Invalid,
    ControllerOn,
    ControllerOff
};

class PPCExample
  : FSM<State>::StateHandler
{
public:
    PPCExample()
      : m_Robot(MaxSpeed, 500_mm, 0_m, MaxTurn)
      , m_Controller(LookAheadDistance, m_Robot.getDistanceBetweenAxis(), StoppingDist)
      , m_Display({ 10.2_m, 10.2_m })
      , m_CarSprite(m_Display.createCarAgent(160_mm))
      , m_WayPointLines(m_Display.createLineStrip(sf::Color::Blue))
      , m_LookAheadLine(m_Display.createLineStrip(sf::Color::Red))
      , m_StateMachine(this, Invalid)
    {
        // Add an initial waypoint in the centre of the window
        addWayPoint(sf::Vector2i{ World::WindowWidth / 2, World::WindowHeight / 2 });
    }

    void run()
    {
        // Initially control with keyboard
        m_StateMachine.transition(ControllerOff);

        // Run until window is closed
        while (m_Display.isOpen()) {
            m_StateMachine.update();
        }
    }

private:
    using Event = FSM<State>::StateHandler::Event;

    SimulatedAckermann m_Robot;                 // simulated robot
    Robots::PurePursuitController m_Controller; // for auto-controlling the robot
    World m_Display;                            // display the agent
    World::CarAgent m_CarSprite;
    World::LineStrip m_WayPointLines;
    std::vector<sf::RectangleShape> m_WayPointRectangles;
    World::LineStrip m_LookAheadLine;
    FSM<State> m_StateMachine;

    template<class T>
    void addWayPoint(const T& position)
    {
        constexpr float SquareWidth = 6.f;

        // add rectangle for new way point
        sf::RectangleShape rect{ sf::Vector2f{ SquareWidth, SquareWidth } };
        rect.setFillColor(sf::Color::Blue);
        rect.setPosition(position.x - SquareWidth / 2.f, position.y - SquareWidth / 2.f);
        m_WayPointRectangles.push_back(rect);

        // add point to end of line
        m_WayPointLines.append(sf::Vector2f{ (float) position.x, (float) position.y });

        // tell controller about way point
        m_Controller.addWayPoint(m_Display.pixelToVector(position.x, position.y));
    }

    virtual bool handleEvent(State state, Event event) override
    {
        if (state == Invalid) {
            return true;
        }

        switch (event) {
        case Event::Enter:
            if (state == ControllerOn) {
                LOGI << "Controller ON";

                // reset lookahead distance
                m_Controller.setLookAheadDistance(LookAheadDistance);
            } else {
                LOGI << "Controller OFF";

                // don't draw lookahead line anymore
                m_LookAheadLine.clear();
            }
            break;
        case Event::Exit:
            m_Robot.stopMoving();
            break;
        case Event::Update:
            m_CarSprite.setPose(m_Robot.getPose());

            // update GUI; handle keyboard + mouse events
            auto eventHandler = [&](const sf::Event &event) { handleEvent(state, event); };
            m_Display.drawAndHandleEvents(eventHandler, m_WayPointLines,
                                          m_WayPointRectangles, m_LookAheadLine, m_CarSprite);

            if (state == ControllerOn) {
                const auto lookPoint = m_Controller.getLookAheadPoint(m_Robot.getPose(), LookAheadDistance);

                // draw lookahead point and a line to it from the robot
                m_LookAheadLine.clear();
                if (lookPoint) {
                    m_LookAheadLine.append(m_Robot.getPose());
                    m_LookAheadLine.append(lookPoint.value());
                } else {
                    LOGE << "Robot is stuck! 😭";
                    m_StateMachine.transition(ControllerOff);
                    return true;
                }

                // calculate turning angle with controller
                const auto turningAngle = m_Controller.getTurningAngle(m_Robot.getPose(), lookPoint);
                if (turningAngle) {
                    const auto ang = turningAngle.value();
                    if (abs(ang) > MaxTurn) {
                        m_Robot.move(MaxSpeed, copysign(MaxTurn, ang));
                    } else {
                        m_Robot.move(MaxSpeed, ang);
                    }
                } else {
                    LOGI << "Reached destination!";
                    m_StateMachine.transition(ControllerOff);
                }
            }

            break;
        }

        return true;
    }

    void handleEvent(State state, const sf::Event &event)
    {
        // try driving the robot with the keyboard
        if (state == ControllerOff && Viz::SFML::drive(m_Robot, event)) {
            return;
        }

        // if user presses space, it toggles the controller on/off
        if (event.type == sf::Event::KeyReleased
            && event.key.code == sf::Keyboard::Space)
        {
            if (state == ControllerOn) {
                m_StateMachine.transition(ControllerOff);
            } else if (m_WayPointRectangles.size() > 1) {
                m_StateMachine.transition(ControllerOn);
            } else {
                LOGW << "Too few way points!";
            }

            return;
        }

        // the user can add waypoints by clicking
        if (event.type == sf::Event::MouseButtonPressed
            && event.mouseButton.button == sf::Mouse::Left)
        {
            addWayPoint(event.mouseButton);
        }
    }
};

int
bobMain(int, char **)
{
    PPCExample ppc;
    ppc.run();

    return EXIT_SUCCESS;
}
