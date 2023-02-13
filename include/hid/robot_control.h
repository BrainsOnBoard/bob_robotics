#pragma once

// BoB robotics includes
#include "common/circstat.h"
#include "hid/joystick.h"
#include "robots/type_traits.h"

// Standard C++ includes
#include <algorithm>
#include <array>
#include <type_traits>

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace HID {
constexpr float DefaultDeadZone = 0.25f;
constexpr float DefaultGain = 1.f;

template<class RobotType>
void
drive(RobotType &robot, const JoystickBase<JAxis, JButton> &joystick,
      float deadZone = DefaultDeadZone)
{
    drive(robot, joystick, deadZone, DefaultGain);
}

template<class RobotType>
void addJoystick(RobotType &robot, JoystickBase<JAxis, JButton> &joystick,
                 float deadZone = DefaultDeadZone)
{
    addJoystick(robot, joystick, deadZone, DefaultGain);
}

template<class AckermannType,
         std::enable_if_t<Robots::IsAckermann<AckermannType>::value, int> = 0>
constexpr auto usedAxes()
{
    return std::array<JAxis, 2>{ JAxis::LeftStickVertical,
                                 JAxis::RightStickHorizontal };
}

template<class Omni2DType,
         std::enable_if_t<Robots::IsOmni2D<Omni2DType>::value, int> = 0>
constexpr auto usedAxes()
{
    return std::array<JAxis, 3>{ JAxis::LeftStickVertical,
                                 JAxis::LeftStickHorizontal,
                                 JAxis::RightStickHorizontal };
}

template<class TankType,
         std::enable_if_t<Robots::IsTank<TankType>::value, int> = 0>
constexpr auto usedAxes()
{
    return std::array<JAxis, 2>{ JAxis::LeftStickVertical,
                                 JAxis::LeftStickHorizontal };
}

/**!
 * \brief Add a handler to control a robot with a joystick
 *
 * Save boilerplate by writing this function generically.
 */
template<class RobotType,
         std::enable_if_t<!Robots::IsUAV<RobotType>::value, int> = 0>
void
addJoystick(RobotType &robot, JoystickBase<JAxis, JButton> &joystick,
            float deadZone, float gain)
{
    joystick.addHandler([&robot, deadZone, gain](auto &joystick, JAxis axis, float) {
        constexpr auto axes = usedAxes<RobotType>();
        if (std::find(axes.begin(), axes.end(), axis) != axes.end()) {
            drive(robot, joystick, deadZone, gain);
            return true;
        }

        return false;
    });
}

template<class AckermannType, std::enable_if_t<Robots::IsAckermann<AckermannType>::value, int> = 0>
void
drive(AckermannType &robot, JoystickBase<JAxis, JButton> &joystick,
      float deadZone, float gain)
{
    const auto thresh = [deadZone, gain](float val) {
        return fabs(val) <= deadZone ? 0.f : val * gain;
    };

    robot.move(thresh(joystick.getState(JAxis::LeftStickVertical)),
               thresh(joystick.getState(JAxis::RightStickHorizontal)));
}

template<class Omni2DType, std::enable_if_t<Robots::IsOmni2D<Omni2DType>::value, int> = 0>
void
drive(Omni2DType &robot, const JoystickBase<JAxis, JButton> &joystick,
      float deadZone, float gain)
{
    const auto forward = -joystick.getState(JAxis::LeftStickVertical);
    const auto sideways = joystick.getState(JAxis::LeftStickHorizontal);
    const auto turn = joystick.getState(JAxis::RightStickHorizontal);

    const bool deadForward = (fabs(forward) < deadZone);
    const bool deadSideways = (fabs(sideways) < deadZone);
    const bool deadTurn = (fabs(turn) < deadZone);

    robot.omni2D(gain * forward * !deadForward, gain * sideways * !deadSideways,
                 gain * turn * !deadTurn);
}

template<class TankType, std::enable_if_t<Robots::IsTank<TankType>::value, int> = 0>
void
drive(TankType &robot, const JoystickBase<JAxis, JButton> &joystick,
      float deadZone, float gain)
{
    const auto x = gain * joystick.getState(JAxis::LeftStickHorizontal);
    const auto y = gain * joystick.getState(JAxis::LeftStickVertical);
    constexpr float halfPi = pi<float>() / 2.0f;

    const bool deadX = (fabs(x) < deadZone);
    const bool deadY = (fabs(y) < deadZone);
    if (deadX && deadY) {
        robot.tank(0.0f, 0.0f);
    } else if (deadX) {
        robot.tank(-y, -y);
    } else if (deadY) {
        robot.tank(x, -x);
    } else {
        // If length of joystick vector places it in deadZone, stop motors
        float r = hypot(x, y);

        // By removing deadzone, we're preventing it being possible to drive at low speed
        // So subtract deadzone, rescale the result and clamp so it's back on (0,1)
        r = std::min(1.f, (r - deadZone) / (1.0f - deadZone));

        const float theta = atan2(x, -y);
        const float twoTheta = 2.0f * theta;

        // Drive motor
        if (theta >= 0.0f && theta < halfPi) {
            robot.tank(r, r * cos(twoTheta));
        } else if (theta >= halfPi && theta < pi<float>()) {
            robot.tank(-r * cos(twoTheta), -r);
        } else if (theta < 0.0f && theta >= -halfPi) {
            robot.tank(r * cos(twoTheta), r);
        } else if (theta < -halfPi && theta >= -pi<float>()) {
            robot.tank(-r, -r * cos(twoTheta));
        }
    }
}

template<class TankType, std::enable_if_t<Robots::IsTank<TankType>::value, int> = 0>
void
controlWithThumbsticks(TankType &robot, JoystickBase<JAxis, JButton> &joystick)
{
    joystick.addHandler(
            [&robot](auto &joystick, JAxis axis, float) {
                constexpr auto Left = JAxis::LeftStickVertical;
                constexpr auto Right = JAxis::RightStickVertical;
                if (axis == Left || axis == Right) {
                    robot.tank(joystick.getState(Left), joystick.getState(Right));
                    return true;
                }

                return false;
            });
}

template<class UAVType, std::enable_if_t<Robots::IsUAV<UAVType>::value, int> = 0>
void
drive(UAVType &uav, const JoystickBase<JAxis, JButton> &joystick,
      float /*deadZone*/, float gain)
{
    uav.setRoll(gain * joystick.getState(JAxis::RightStickHorizontal));
    uav.setPitch(gain * -joystick.getState(JAxis::RightStickVertical));
    uav.setVerticalSpeed(gain * -joystick.getState(JAxis::LeftStickVertical));
    uav.setYawSpeed(gain * (joystick.getState(JAxis::RightTrigger) - joystick.getState(JAxis::LeftTrigger)));
}

template<class UAVType, std::enable_if_t<Robots::IsUAV<UAVType>::value, int> = 0>
void
addJoystick(UAVType &uav, JoystickBase<JAxis, JButton> &joystick,
            float /*deadZone*/, float gain)
{
    auto onAxisEvent = [&uav, gain](auto &, JAxis axis, float value) {
        /*
         * setRoll/Pitch etc. all take values between -1 and 1. We cap these
         * values for the joystick code to make the drone more controllable.
         */
        switch (axis) {
        case JAxis::RightStickHorizontal:
            uav.setRoll(gain * value);
            return true;
        case JAxis::RightStickVertical:
            uav.setPitch(gain * -value);
            return true;
        case JAxis::LeftStickVertical:
            uav.setVerticalSpeed(gain * -value);
            return true;
        case JAxis::LeftTrigger:
            uav.setYawSpeed(gain * -value);
            return true;
        case JAxis::RightTrigger:
            uav.setYawSpeed(gain * value);
            return true;
        default:
            // otherwise signal that we haven't handled event
            return false;
        }
    };

    auto onButtonEvent = [&uav](auto &, JButton button, bool pressed) {
        // we only care about button presses
        if (!pressed) {
            return false;
        }

        // A = take off; B = land
        switch (button) {
        case JButton::A:
            uav.takeOff();
            return true;
        case JButton::B:
            uav.land();
            return true;
        default:
            // otherwise signal that we haven't handled event
            return false;
        }
    };

    joystick.addHandler(onAxisEvent);
    joystick.addHandler(onButtonEvent);
}
} // HID
} // BoBRobotics
