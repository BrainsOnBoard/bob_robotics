#pragma once

// BoB robotics includes
#include "camera.h"
#include "common/pose.h"
#include "hid/joystick.h"
#include "renderer.h"
#include "robots/robot.h"

// Standard C++ includes
#include <mutex>

namespace BoBRobotics {
namespace AntWorld {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::AntAgent
//----------------------------------------------------------------------------
//! A simple agent with a position and a panoramic view of the current AntWorld
class AntAgent
  : public Robots::Robot
  , public Camera
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    // I chose these values to be (roughly) desert ant-like -- AD
    static constexpr meters_per_second_t DefaultVelocity = 0.03_mps;
    static constexpr radians_per_second_t DefaultTurnSpeed = 200_deg_per_s;

    AntAgent(sf::Window &window, Renderer &renderer,
             const cv::Size &renderSize,
             meters_per_second_t velocity = DefaultVelocity,
             radians_per_second_t turnSpeed = DefaultTurnSpeed);

    //----------------------------------------------------------------------------
    // Robot virtuals
    //----------------------------------------------------------------------------
    virtual void moveForward(float speed) override;

    virtual void turnOnTheSpot(float clockwiseSpeed) override;

    virtual void stopMoving() override;

    virtual void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f) override;

    //! Drive the robot using the current joystick state
    virtual void drive(const HID::Joystick &joystick, float deadZone = 0.25f) override;

    //! Add a handler to the connection to drive robot
    virtual void readFromNetwork(Net::Connection &connection) override;

    //! Remove and connection handlers that drive this robot
    virtual void stopReadingFromNetwork() override;

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    radians_per_second_t getMaximumTurnSpeed() const;

    void setPosition(meter_t x, meter_t y, meter_t z);

    void setAttitude(degree_t yaw, degree_t pitch, degree_t roll);

    void updatePose(const units::time::second_t elapsedTime);

private:
    meters_per_second_t m_Velocity;
    radians_per_second_t m_TurnSpeed;
    std::mutex m_PoseMutex;
    float m_JoystickX = 0.f, m_JoystickY = 0.f;

    enum class MoveMode
    {
        NotMoving,
        MovingForward,
        Turning
    } m_MoveMode = MoveMode::NotMoving;

    float m_SpeedProportion; //! From -1 to 1: indicates proportion of max forward/turning speed
}; // AntAgent
} // AntWorld
} // BoBRobotics
