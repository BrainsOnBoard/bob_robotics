#pragma once

namespace BoBRobotics {
namespace Robots {
namespace Ackermann {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::AckermannBase
//----------------------------------------------------------------------------
//! A base class for robots with Ackermann-type steering
template<class Derived>
class AckermannBase
{
protected:
    AckermannBase<Derived>() = default;
}; // AckermannBase
} // Ackermann
} // Robots
} // BoBRobotics
