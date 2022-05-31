#pragma once
#ifndef _WIN32

// BoB robotics includes
#include "serial_interface.h"

// Third-party includes
#include "third_party/units.h"

//----------------------------------------------------------------------------
// BoBRobotics::RoboClaw
//----------------------------------------------------------------------------
namespace BoBRobotics {
class RoboClaw
{
public:
    Roboclaw(const char *path = SerialInterface::DefaultLinuxDevicePath, uint8_t address = 0x80, int maxRetry = 2);

    void setMotor1Speed(float throttle);
    void setMotor2Speed(float throttle);

private:
    //------------------------------------------------------------------------
    // Enumerations
    //------------------------------------------------------------------------
    enum class Error
    {
        NONE            = 0x000000,
        ESTOP           = 0x000001,	//Error: E-Stop active
        TEMP            = 0x000002,	//Error: Temperature Sensor 1 >=100c
        TEMP2           = 0x000004,	//Error: Temperature Sensor 2 >=100C (available only on some models)
        MBATHIGH        = 0x000008,	//Error: Main Battery Over Voltage
        LBATHIGH        = 0x000010,	//Error: Logic Battery High Voltage
        LBATLOW         = 0x000020,	//Error: Logic Battery Low Voltage
        FAULTM1         = 0x000040,	//Error: Motor 1 Driver Fault (only on some models)
        FAULTM2         = 0x000080,	//Error: Motor 2 Driver Fault (only on some models)
        SPEED1          = 0x000100,	//Error: Motor 1 Speed Error Limit
        SPEED2          = 0x000200,	//Error: Motor 2 Speed Error Limit
        POS1            = 0x000400,	//Error: Motor 1 Position Error Limit
        POS2            = 0x000800,	//Error: MOtor2 Position Error Limit
        OVERCURRENTM1   = 0x010000, //Warning: Motor 1 Current Limited
        OVERCURRENTM2   = 0x020000, //Warning: Motor 2 CUrrent Limited
        MBATHIGH        = 0x040000, //Warning: Main Battery Voltage High
        MBATLOW         = 0x080000, //Warning: Main Battery Low Voltage
        TEMP            = 0x100000, //Warning: Temperaure Sensor 1 >=85C
        TEMP2           = 0x200000, //Warning: Temperature Sensor 2 >=85C (available only on some models)
        S4              = 0x400000, //Warning: Motor 1 Home/Limit Signal
        S5              = 0x800000, //Warning: Motor 2 Home/Limit Signal
    };

    enum class Command : uint8_t
    {
        M1FORWARD = 0,
        M1BACKWARD = 1,
        SETMINMB = 2,
        SETMAXMB = 3,
        M2FORWARD = 4,
        M2BACKWARD = 5,
        M17BIT = 6,
        M27BIT = 7,
        MIXEDFORWARD = 8,
        MIXEDBACKWARD = 9,
        MIXEDRIGHT = 10,
        MIXEDLEFT = 11,
        MIXEDFB = 12,
        MIXEDLR = 13,
        GETM1ENC = 16,
        GETM2ENC = 17,
        GETM1SPEED = 18,
        GETM2SPEED = 19,
        RESETENC = 20,
        GETVERSION = 21,
        SETM1ENCCOUNT = 22,
        SETM2ENCCOUNT = 23,
        GETMBATT = 24,
        GETLBATT = 25,
        SETMINLB = 26,
        SETMAXLB = 27,
        SETM1PID = 28,
        SETM2PID = 29,
        GETM1ISPEED = 30,
        GETM2ISPEED = 31,
        M1DUTY = 32,
        M2DUTY = 33,
        MIXEDDUTY = 34,
        M1SPEED = 35,
        M2SPEED = 36,
        MIXEDSPEED = 37,
        M1SPEEDACCEL = 38,
        M2SPEEDACCEL = 39,
        MIXEDSPEEDACCEL = 40,
        M1SPEEDDIST = 41,
        M2SPEEDDIST = 42,
        MIXEDSPEEDDIST = 43,
        M1SPEEDACCELDIST = 44,
        M2SPEEDACCELDIST = 45,
        MIXEDSPEEDACCELDIST = 46,
        GETBUFFERS = 47,
        GETPWMS = 48,
        GETCURRENTS = 49,
        MIXEDSPEED2ACCEL = 50,
        MIXEDSPEED2ACCELDIST = 51,
        M1DUTYACCEL = 52,
        M2DUTYACCEL = 53,
        MIXEDDUTYACCEL = 54,
        READM1PID = 55,
        READM2PID = 56,
        SETMAINVOLTAGES = 57,
        SETLOGICVOLTAGES = 58,
        GETMINMAXMAINVOLTAGES = 59,
        GETMINMAXLOGICVOLTAGES = 60,
        SETM1POSPID = 61,
        SETM2POSPID = 62,
        READM1POSPID = 63,
        READM2POSPID = 64,
        M1SPEEDACCELDECCELPOS = 65,
        M2SPEEDACCELDECCELPOS = 66,
        MIXEDSPEEDACCELDECCELPOS = 67,
        SETM1DEFAULTACCEL = 68,
        SETM2DEFAULTACCEL = 69,
        SETPINFUNCTIONS = 74,
        GETPINFUNCTIONS = 75,
        SETDEADBAND	= 76,
        GETDEADBAND	= 77,
        GETENCODERS = 78,
        GETISPEEDS = 79,
        RESTOREDEFAULTS = 80,
        GETTEMP = 82,
        GETTEMP2 = 83,	//Only valid on some models
        GETERROR = 90,
        GETENCODERMODE = 91,
        SETM1ENCODERMODE = 92,
        SETM2ENCODERMODE = 93,
        WRITENVM = 94,
        READNVM = 95,	//Reloads values from Flash into Ram
        SETCONFIG = 98,
        GETCONFIG = 99,
        SETM1MAXCURRENT = 133,
        SETM2MAXCURRENT = 134,
        GETM1MAXCURRENT = 135,
        GETM2MAXCURRENT = 136,
        SETPWMMODE = 148,
        GETPWMMODE = 149,
        FLAGBOOTLOADER = 255    //Only available via USB communications
    };

    //------------------------------------------------------------------------
    // CRC
    //------------------------------------------------------------------------
    //! CRC implementation, copied from Arduino code
    class CRC
    {
        CRC() : m_CRC(0)
        {
        }

        void update(uint8_t data)
        {
            m_CRC = m_CRC ^ ((uint16_t)data << 8);
            for(int i = 0; i < 8; i++) {
                if (m_CRC & 0x8000) {
                    m_CRC = (m_CRC << 1) ^ 0x1021;
                }
                else {
                    m_CRC <<= 1;
                }
            }
        }

        uint16_t get()
        {
            return m_CRC;
        }

    private:
        uint16_t m_CRC;
    };

    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    template<typename ...Data>
    void write(CRC&)
    {
    }

    template<typename ...Data>
    void write(CRC &crc,
               uint8_t byte, Data... data)
    {
        // Write byte and update CRC
        m_SerialInterface.writeByte(byte);
        crc.update(byte);

        // Write remainder
        write(crc, data...);
    }

    template<typename ...Data>
    void write(CRC &crc,
               uint32_t dword, Data... data)
    {
        // Write bytes of dword followed by remainder
        // **NOTE** RoboClaw is big endian so SerialInterface::write wouldn't work
        write(crc, (uint8_t)(dword >> 24), (uint8_t)(dword >> 16), (uint8_t)(dword >> 8),(uint8_t)dword, data...);
    }

    template<typename ...Data>
    void write(CRC &crc,
               uint8_t word, Data... data)
    {
        // Write bytes of word followed by remainder
        // **NOTE** RoboClaw is big endian so SerialInterface::write wouldn't work
        write(crc, (uint8_t)(word >> 8),(uint8_t)word, data...);
    }

    template<typename ...Data>
    void writeN(Command command, Data... data)
    {
        for(int r = 0; r < maxRetry; r++) {
            // Start CRC calculation
            CRC crc;

            // Write data
            write(crc, m_Address, static_cast<uint8_t>(command), data...);

            // Write CRC
            // **NOTE** RoboClaw is big endian so SerialInterface::write wouldn't work
            const uint16_t crc = crc.get();
            m_SerialInterface.writeByte(crc >> 8);
            m_SerialInterface.writeByte(crc);

            // If byte is read successfully
            uint8_t response;
            if(m_SerialInterface.readByte(reponse) && response == 0xFF) {
                return;
            }
        }

        throw std::runtime_error("Unable to write to serial port");
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    //! Serial interface for communicating with RoboClaw
    SerialInterface m_SerialInterface;

    //! Address of RoboClaw we're targetting
    const uint8_t m_Address;

    //! Number of times to retry failed reads and writes
    const int m_MaxRetry;
};
}   // namespace BoBRobotics

#endif   // _WIN32
