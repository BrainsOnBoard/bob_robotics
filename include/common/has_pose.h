#pragma once

namespace BoBRobotics {
template<class Derived>
class HasPose {
public:
    auto getAttitude() {
        auto derived = reinterpret_cast<Derived *>(this);
        return derived->getPose().attitude();
    }

    auto getPosition() {
        auto derived = reinterpret_cast<Derived *>(this);
        return derived->getPose().position();
    }
}; // HasPose
} // BoBRobotics
