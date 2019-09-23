#pragma once

// BoB robotics includes
#include "has_pose.h"
#include "pose.h"

namespace BoBRobotics {
//! A dummy class which returns all zeros for its pose
class DummyPoseGetter
  : public HasPose<DummyPoseGetter> {
public:
    static constexpr auto getPose()
    {
        return Pose3<units::length::meter_t, units::angle::radian_t>{};
    }
};
}
