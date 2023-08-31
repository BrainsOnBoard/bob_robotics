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
    Omni2DBase()
    :   m_Forward(0.0f), m_Sideways(0.0f), m_Turn(0.0f)
    {}

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

    float getForward() const{ return m_Forward; }
    float getSideways() const{ return m_Sideways; }
    float getTurn() const{ return m_Turn; }

    //! Get forward movement speed
    float getForwardSpeed() const
    {
        return getForward();
    }

    //! Get turning speed
    float getTurnSpeed() const
    {
        return getTurn();
    }

private:
    void omni2D(float forward, float sideways, float turn)
    {
        auto *derived = static_cast<Derived *>(this);
        m_Forward = forward;
        m_Sideways = sideways;
        m_Turn = turn;
        derived->omni2D(forward, sideways, turn);
    }

    float m_Forward;
    float m_Sideways;
    float m_Turn;

}; // Omni2D
} // Omni2D
} // Robots
} // BoBRobotics
