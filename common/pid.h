#pragma once

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
//------------------------------------------------------------------------
// BoBRobotics::PID
//------------------------------------------------------------------------
template<typename T = float>
class PID
{
public:
    PID(T kp, T ki, T kd, T outMin, T outMax)
    :   m_Intergral(0.0f), m_KP(kp), m_KI(ki), m_KD(kd), m_OutMin(outMin), m_OutMax(outMax)
    {}

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // (Re-)initialise PID - use before output of PID is connected to plant
    void initialise(T input, T output)
    {
        m_LastInput = input;
        m_Intergral = output;
        m_Intergral = std::min(m_OutMax, std::max(m_OutMin, m_Intergral));
    }

    // Get output based on setpoint
    T update(T setpoint, T input)
    {
        const T error = setpoint - input;

        // Update integral term and clamp
        m_Intergral += (m_KI * error);
        m_Intergral = std::min(m_OutMax, std::max(m_OutMin, m_Intergral));

        // Calculate derivative term
        const T derivative = input - m_LastInput;

        // Calculate output and clamp
        T output = (m_KP * error) + m_Intergral - (m_KD * derivative);
        output = std::min(m_OutMax, std::max(m_OutMin, output));

        // Update last input
        m_LastInput = input;

        return output;
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Last input (used for calculating derivative)
    T m_LastInput;

    // Integral
    T m_Intergral;

    // PID constants
    T m_KP;
    T m_KI;
    T m_KD;

    // Output range
    const T m_OutMin;
    const T m_OutMax;
};
} // BoBRobotics
