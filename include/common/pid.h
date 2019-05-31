#pragma once

namespace BoBRobotics {
//------------------------------------------------------------------------
// BoBRobotics::PID
//------------------------------------------------------------------------
class PID
{
public:
    PID(float kp, float ki, float kd, float outMin, float outMax);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // (Re-)initialise PID - use before output of PID is connected to plant
    void initialise(float input, float output);

    // Get output based on setpoint
    float update(float setpoint, float input);

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
} // BoBRobotics
