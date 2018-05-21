#pragma once

// Standard C++ includes
#include <algorithm>

namespace GeNN_Robotics {
//------------------------------------------------------------------------
// PID
//------------------------------------------------------------------------
class PID
{
public:
    PID(float kp, float ki, float kd, float outMin, float outMax) 
    :   m_Intergral(0.0f), m_KP(kp), m_KI(ki), m_KD(kd), m_OutMin(outMin), m_OutMax(outMax)
    {
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // (Re-)initialise PID - use before output of PID is connected to plant
    void initialise(float input, float output)
    {
        m_LastInput = input;
        m_Intergral = output;
        m_Intergral = std::min(m_OutMax, std::max(m_OutMin, m_Intergral));
    }
    
    // Get output based on setpoint
    float update(float setpoint, float input)
    {
        const float error = setpoint - input;
        
        // Update integral term and clamp
        m_Intergral += (m_KI * error);
        m_Intergral = std::min(m_OutMax, std::max(m_OutMin, m_Intergral));
        
        // Calculate derivative term
        const float derivative = input - m_LastInput;
        
        // Calculate output and clamp
        float output = (m_KP * error) + m_Intergral - (m_KD * derivative);
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
    float m_LastInput;
    
    // Integral
    float m_Intergral;
    
    // PID constants
    float m_KP;
    float m_KI;
    float m_KD;
    
    // Output range
    const float m_OutMin;
    const float m_OutMax;
};
} // GeNN_Robotics
