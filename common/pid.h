#pragma once

// BoB robotics includes
#include "assert.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <limits>

namespace BoBRobotics {
//------------------------------------------------------------------------
// BoBRobotics::PID
//------------------------------------------------------------------------
class PID
{
public:
    PID(float kp, float ki, float kd, float outMin, float outMax)
      : m_Integral(std::numeric_limits<float>::quiet_NaN())
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
    void initialise(float input, float output)
    {
        m_LastInput = input;
        m_Integral = output;
        m_Integral = std::min(m_OutMax, std::max(m_OutMin, m_Integral));
    }

    // Get output based on setpoint
    float update(float setpoint, float input, bool debug)
    {
        BOB_ASSERT(isInitialised());

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

        if (debug)
            std::cout << "error: " << error << " | output: " << output << std::endl;

        return output;
    }

    bool isInitialised() const { return !std::isnan(m_Integral); }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Last input (used for calculating derivative)
    float m_LastInput;

    // Integral
    float m_Integral;

    // PID constants
    float m_KP;
    float m_KI;
    float m_KD;

    // Output range
    const float m_OutMin;
    const float m_OutMax;
};
} // BoBRobotics
