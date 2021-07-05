#pragma once

// BoB robotics includes
#include "hid/joystick.h"
#include "net/connection.h"
#include "tank.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2D
//----------------------------------------------------------------------------
// Interface for driving Omni2D-like wheeled robots
class Omni2D : public Tank
{
public:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual void omni2D(float forward, float sideways, float turn);

    //------------------------------------------------------------------------
    // Robot virtuals
    //------------------------------------------------------------------------
    virtual void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f) override;
    virtual void drive(const HID::Joystick &joystick, float deadZone = 0.25f) override;
    virtual void readFromNetwork(Net::Connection &connection) override;
    virtual void stopReadingFromNetwork() override;

    //------------------------------------------------------------------------
    // Tank virtuals
    //------------------------------------------------------------------------
    virtual void tank(float left, float right) override;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    float getForwards() const;
    float getSideways() const;
    float getTurn() const;

protected:
    void setWheelSpeed(float forward, float sideways, float turn);

private:
    float m_Forward = 0;
    float m_Sideways = 0;
    float m_Turn = 0;
    Net::Connection *m_Connection = nullptr;

    void drive(float forward, float sideways, float turn, float deadZone);
    void onOmniCommandReceived(Net::Connection &, const Net::Command &command);
    bool onJoystickEvent(HID::JAxis axis, float value, float deadZone);
}; // Omni2D
} // Robots
} // BoBRobotics
