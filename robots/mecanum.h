#pragma once

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

// Common includes
#include "../common/serial_interface.h"
#include "omni2D.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Mecanum
//----------------------------------------------------------------------------
class Mecanum : public Omni2D
{
public:
    Mecanum(const char *path = "/dev/ttyACM0")
      : m_Serial(path)
      , m_Forward(0.0f)
      , m_Sideways(0.0f)
      , m_Turn(0.0f)
    {}

    //----------------------------------------------------------------------------
    // Omni2D virtuals
    //----------------------------------------------------------------------------
    virtual void omni2D(float forwards, float sideways, float turn) override
    {
        // Cache left and right
        m_Forward = forwards;
        m_Sideways = sideways;
        m_Turn = turn;
        
		// resolve to motor speeds
		float m1 = (-m_Sideways - m_Forward - m_Turn); 
		float m2 = (-m_Sideways + m_Forward + m_Turn); 
		float m3 = (-m_Sideways + m_Forward - m_Turn); 
		float m4 = (-m_Sideways - m_Forward + m_Turn); 
		
		//std::cout << m1 << m2 << m3 << m4 << std::endl;
		
		// bounds checking after resolving...
		m1 = m1>1.0f ? 1.0f : m1;
		m2 = m2>1.0f ? 1.0f : m2;
		m3 = m3>1.0f ? 1.0f : m3;
		m4 = m4>1.0f ? 1.0f : m4;
		m1 = m1<-1.0f ? -1.0f : m1;
		m2 = m2<-1.0f ? -1.0f : m2;
		m3 = m3<-1.0f ? -1.0f : m3;
		m4 = m4<-1.0f ? -1.0f : m4;
		
		uint8_t buffer[9] = {m1>0, m2>0, m3>0, m4>0, (uint8_t) (fabs(m1)*254.0f), (uint8_t) (fabs(m2)*254.0f), (uint8_t) (fabs(m3)*254.0f), (uint8_t) (fabs(m4)*254.0f), 255};

        // Send buffer
        write(buffer);

    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        m_Serial.read(data);
    }

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_Serial.write(data);
    }

    float getForwards() const
    {
        return m_Forward;
    }

    float getSideways() const
    {
        return m_Sideways;
    }

    float getTurn() const
    {
        return m_Turn;
    }

private:
    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    uint8_t floatToI2C(float speed)
    {
        return (uint8_t) std::min(
                255.0f,
                std::max(0.0f, std::round(((speed + 1.0f) / 2.0f) * 255.0f)));
    }

    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    BoBRobotics::SerialInterface m_Serial;
    float m_Forward;
    float m_Sideways;
    float m_Turn;
}; // Mecanum
} // Robots
} // BoBRobotics
