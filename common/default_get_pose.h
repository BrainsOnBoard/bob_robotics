#pragma once

// BoB robotics includes
#include "pose.h"

namespace BoBRobotics {
template<typename Derived, typename DefaultLengthUnit, typename DefaultAngleUnit>
class DefaultGetPose
{
public:
    template<typename LengthUnit = DefaultLengthUnit, typename AngleUnit = DefaultAngleUnit>
    auto getPose()
    {
        auto derived = reinterpret_cast<Derived *>(this);
        return Pose3<LengthUnit, AngleUnit>(derived->template getPosition<LengthUnit>(),
                                            derived->template getAttitude<AngleUnit>());
    }
}; // DefaultGetPose
} // BoBRobotics
