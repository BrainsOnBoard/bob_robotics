#pragma once

// BoB robotics includes
#include "robots/tank/tank_base.h"

namespace BoBRobotics {
namespace Robots {
namespace Omni2D {
template<class Omni2DType>
class TankAdaptor : public Tank::TankBase<TankAdaptor<Omni2DType>>
{
    friend Tank::TankBase<TankAdaptor<Omni2DType>>
public:
    TankAdaptor(Omni2DType &omni)
      : m_Omni{ omni }
    {}

private:
    //! Implement tank controls in terms of omni
    void tankInternal(float left, float right)
    {
        m_Omni.omni2D((left + right) / 2.0f, 0.0f, (left - right) / 2.0f);
    }

    Omni2DType &m_Omni;
}; // TankAdaptor
} // Omni2D
} // Robots
} // BoBRobotics
