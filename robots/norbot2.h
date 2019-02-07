
#pragma once

// BoB robotics includes
#include "../common/i2c_interface.h"


// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <vector>

// third party includes
#include "../third_party/units.h"


//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//! An interface for 4 wheeled, Arduino-based robots developed at the University of Sussex
class Norbot2 
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;
    

public:
    Norbot2(const char *path = "/dev/i2c-1", int slaveAddress = 0x29)
      : m_I2C(path, slaveAddress)
      , m_forwardVelocity(0.0f)
      , m_turningAngle(0.0f)
    {}

    // destructor		
    ~Norbot2() 
    {
        stopMoving();
    }

    void move(uint8_t speed, uint8_t turningAngle)
    {
      
        // Cache left and right
        m_forwardVelocity = speed;      // 0-127 backward, 128-255 forward
        m_turningAngle = turningAngle;  // 55 left, 90 center, 125 right

        // Convert standard (-1,1) values to bytes in order to send to I2C slave
        uint8_t buffer[2] = { speed, turningAngle };

        // Send buffer
        write(buffer);
    }
 
    // stops the car 
    void stopMoving() {
        move(127, 90);
    }
   

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        m_I2C.read(data);
    }

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_I2C.write(data);
    }

    float getForwardVelocity() const
    {
        return m_forwardVelocity;
    }

    float getTurningAngle() const
    {
        return m_turningAngle;
    }

private:
    
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------

    BoBRobotics::I2CInterface m_I2C;
    uint8_t m_forwardVelocity;
    uint8_t m_turningAngle;

}; // Norbot2


