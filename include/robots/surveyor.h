#pragma once

// BoB robotics includes
#include "tank.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Surveyor
//----------------------------------------------------------------------------
/*!
 * \brief An interface for Active Robots' Surveyor line of robots
 *
 * See https://www.active-robots.com/surveyor-rover.html
 */
class Surveyor : public Tank
{
    using millimeter_t = units::length::millimeter_t;

public:
    Surveyor(const std::string &address, uint16_t port);

    virtual ~Surveyor() override;

    //----------------------------------------------------------------------------
    // Tank virtuals
    //----------------------------------------------------------------------------
    virtual void tank(float left, float right) override;

    virtual millimeter_t getRobotWidth() const override;

private:
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    int m_Socket;
}; // Surveyor
} // Robots
} // BoBRobotics
