// BoB robotics includes
#include "common/pid.h"

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
PID::PID(float kp, float ki, float kd, float outMin, float outMax)
  : m_Integral(0.0f)
  , m_KP(kp)
  , m_KI(ki)
  , m_KD(kd)
  , m_OutMin(outMin)
  , m_OutMax(outMax)
{
}

//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
// (Re-)initialise PID - use before output of PID is connected to plant
void
PID::initialise(float input, float output)
{
    m_LastInput = input;
    m_Integral = output;
    m_Integral = std::min(m_OutMax, std::max(m_OutMin, m_Integral));
}

// Get output based on setpoint
float
PID::update(float setpoint, float input)
{
    const float error = setpoint - input;

    // Update integral term and clamp
    m_Integral += (m_KI * error);
    m_Integral = std::min(m_OutMax, std::max(m_OutMin, m_Integral));

    // Calculate derivative term
    const float derivative = input - m_LastInput;

    // Calculate output and clamp
    float output = (m_KP * error) + m_Integral - (m_KD * derivative);
    output = std::min(m_OutMax, std::max(m_OutMin, output));

    // Update last input
    m_LastInput = input;

    return output;
}
} // BoBRobotics
