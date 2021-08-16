#pragma once

namespace BoBRobotics {
namespace Robots {
namespace UAV {
template<class Derived>
class UAVBase
{
protected:
    UAVBase<Derived>() = default;

public:
    void moveForward(float speed)
    {
        setPitch(speed);
    }

    void turnOnTheSpot(float clockwiseSpeed)
    {
        setPitch(0.f);
        setRoll(0.f);
        setVerticalSpeed(0.f);
        setYawSpeed(clockwiseSpeed);
    }

    void stopMoving()
    {
        setPitch(0.f);
        setRoll(0.f);
        setVerticalSpeed(0.f);
        setYawSpeed(0.f);
    }

private:
#define DERIVED_FUNC(NAME)                         \
    void NAME(float value)                         \
    {                                              \
        static_cast<Derived *>(this)->NAME(value); \
    }

    DERIVED_FUNC(setPitch)
    DERIVED_FUNC(setRoll)
    DERIVED_FUNC(setYawSpeed)
    DERIVED_FUNC(setVerticalSpeed)

#undef DERIVED_FUNC
}; // UAVBase
} // UAV
} // Robots
} // BoBRobotics
