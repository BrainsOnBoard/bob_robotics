#pragma once

// BoB robotics includes
#include "hid/joystick.h"
#include "net/connection.h"
#include "robot.h"

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank
//----------------------------------------------------------------------------
//! Interface for driving wheeled robots with tank steering
class Tank
  : public Robot
{
/*
 * If these are declared private then they annoyingly conflict with "usings" in
 * derived classes.
 */
protected:
    using meter_t = units::length::meter_t;
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    virtual void moveForward(float speed) override;

    virtual void turnOnTheSpot(float clockwiseSpeed) override;

    virtual void stopMoving() override;

    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f);

    void controlWithThumbsticks(HID::Joystick &joystick);

    void drive(const HID::Joystick &joystick, float deadZone = 0.25f);

    void move(meters_per_second_t v,
              radians_per_second_t clockwiseSpeed,
              const bool maxScaled = false);

    //! Set the left and right motors to the specified speed
    virtual void tank(float left, float right);

    void tankMaxScaled(const float left, const float right, const float max = 1.f);

    void tank(meters_per_second_t left, meters_per_second_t right, bool maxScaled = false);

    virtual millimeter_t getRobotWidth() const;

    virtual meters_per_second_t getMaximumSpeed() const;

    virtual meters_per_second_t getAbsoluteMaximumSpeed() const;

    virtual radians_per_second_t getMaximumTurnSpeed() const;

    virtual radians_per_second_t getAbsoluteMaximumTurnSpeed() const;

    virtual void setMaximumSpeedProportion(float value);

    virtual float getMaximumSpeedProportion() const;

    //! Controls the robot with a network stream
    void readFromNetwork(Net::Connection &connection);

    void stopReadingFromNetwork();

    float getLeft() const;

    float getRight() const;

private:
    Net::Connection *m_Connection = nullptr;
    float m_X = 0, m_Y = 0, m_MaximumSpeedProportion = 1.f, m_Left = 0.f, m_Right = 0.f;

    void drive(float x, float y, float deadZone);

    void onCommandReceived(Net::Connection &, const Net::Command &command);

    bool onJoystickEvent(HID::JAxis axis, float value, float deadZone);

protected:
    void setWheelSpeeds(float left, float right);

}; // Tank
} // Robots
} // BoBRobotics

// Include appropriate header, depending on what kind of tank robot the user wants
#if defined(TANK_TYPE_Norbot)
#include "norbot.h"
#elif defined(TANK_TYPE_EV3)
#include "ev3/ev3.h"
#elif defined(TANK_TYPE_Surveyor)
#include "surveyor.h"
#elif defined(TANK_TYPE_BundledTankNetSink)
#include "tank_netsink.h"
#endif
