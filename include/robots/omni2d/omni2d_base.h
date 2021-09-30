#pragma once

namespace BoBRobotics {
namespace Robots {
namespace Omni2D {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2D::Omni2DBase
//----------------------------------------------------------------------------
// Interface for driving Omni2D-like wheeled robots
template<class Derived>
class Omni2DBase
{
protected:
    Omni2DBase<Derived>() = default;

public:
    void moveForward(float speed)
    {
        omni2D(speed, 0.f, 0.f);
    }

    void turnOnTheSpot(float clockwiseSpeed)
    {
        // **TODO**: Check that this really is clockwise
        omni2D(0.f, 0.f, clockwiseSpeed);
    }

    void stopMoving()
    {
        omni2D(0.f, 0.f, 0.f);
    }

private:
    void omni2D(float forward, float sideways, float turn)
    {
        auto *derived = static_cast<Derived *>(this);
        derived->omni2D(forward, sideways, turn);
    }

}; // Omni2D
} // Omni2D
} // Robots
} // BoBRobotics
