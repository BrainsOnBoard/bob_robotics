#pragma once

// Standard C++ includes
#include <utility>

namespace BoBRobotics {
template<typename Derived, typename DefaultLengthUnit, typename DefaultAngleUnit>
class DefaultGetPose
{
    public:
    template<typename LengthUnit = DefaultLengthUnit, typename AngleUnit = DefaultAngleUnit>
    auto getPose()
    {
        auto derived = reinterpret_cast<Derived *>(this);
        return std::make_pair(derived->template getPosition<LengthUnit>(),
                              derived->template getAttitude<AngleUnit>());
    }
}; // DefaultGetPose
} // BoBRobotics
