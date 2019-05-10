#pragma once

namespace BoBRobotics {
namespace Robots {
template<class Derived>
class PositionerBase
{
public:
    template<class PoseType, class Func>
    bool moveToSync(const PoseType &pose, Func extraCalls)
    {
        auto derived = static_cast<Derived *>(this);
        derived->moveTo(pose);
        while (derived->pollPositioner()) {
            if (!extraCalls()) {
                return false;
            }
        }
        derived->getRobot().stopMoving();
        return true;
    }
};
} // Robots
} // BoBRobotics
