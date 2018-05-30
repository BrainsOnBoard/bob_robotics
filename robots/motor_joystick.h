#pragma once

// C includes
#include <cmath>

// GeNN robotics includes
#include "../hid/joystick.h"

// local includes
#include "motor.h"

namespace GeNNRobotics {
namespace Robots {
class MotorJoystick : public HID::Joystick
{
public:
    MotorJoystick(Motor &motor)
      : MotorJoystick(&motor)
    {}

    MotorJoystick(Motor *motor)
      : m_Motor(motor)
    {
        addHandler([=](HID::Event &js) { return joystickCallback(js); });
    }

private:
    Motor *m_Motor;
    float m_X = 0;
    float m_Y = 0;

    bool joystickCallback(HID::Event &js)
    {
        // only interested in left joystick
        float x = m_X;
        float y = m_Y;
        switch (js.axis()) {
        case HID::Axis::LeftStickVertical:
            y = js.axisValue();
            break;
        case HID::Axis::LeftStickHorizontal:
            x = js.axisValue();
            break;
        default:
            return false;
        }

        // Code below is adapted from Jamie's joystick.h - AD
        // If length of joystick vector places it in deadzone, stop motors
        const float r = sqrt((x * x) + (y * y));
        const float theta = atan2(x, -y);
        const float twoTheta = 2.0f * theta;

        // Drive motor
        const float pi = 3.141592653589793238462643383279502884f;
        if (theta >= 0.0f && theta < pi / 2) {
            m_Motor->tank(r, r * cos(twoTheta));
        } else if (theta >= pi / 2 && theta < pi) {
            m_Motor->tank(-r * cos(twoTheta), -r);
        } else if (theta < 0.0f && theta >= -pi / 2) {
            m_Motor->tank(r * cos(twoTheta), r);
        } else if (theta < -pi / 2 && theta >= -pi) {
            m_Motor->tank(-r, -r * cos(twoTheta));
        }

        // signal that we have handled the event
        return true;
    }
};
}
}
