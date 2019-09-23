#pragma once

// BoB robotics includes
#ifdef POSE_GETTER_HEADER
#include POSE_GETTER_HEADER
#endif

// Standard C++ includes
#ifdef POSE_GETTER_TYPE
#include <memory>
#endif

namespace BoBRobotics {
template<class PoseGetter>
auto createPoseGetter(PoseGetter &getter) {
#ifdef POSE_GETTER_HEADER
    (void) getter; // unused
    return std::make_unique<POSE_GETTER_BASE_TYPE>();
#else
    return &getter;
#endif
}
} // BoBRobotics
