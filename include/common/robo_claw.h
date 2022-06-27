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
    RoboClaw(const char *path = SerialInterface::DefaultLinuxDevicePath, uint8_t address = 0x80);

    //------------------------------------------------------------------------
    // Enumerations
    //------------------------------------------------------------------------
    enum class Status : uint32_t
    {
        NONE                = 0x000000,
        ERROR_ESTOP         = 0x000001, //Error: E-Stop active
        ERROR_TEMP          = 0x000002, //Error: Temperature Sensor 1 >=100c
        ERROR_TEMP2         = 0x000004, //Error: Temperature Sensor 2 >=100C (available only on some models)
        ERROR_MBATHIGH      = 0x000008, //Error: Main Battery Over Voltage
        ERROR_LBATHIGH      = 0x000010, //Error: Logic Battery High Voltage
        ERROR_LBATLOW       = 0x000020, //Error: Logic Battery Low Voltage
        ERROR_FAULTM1       = 0x000040, //Error: Motor 1 Driver Fault (only on some models)
        ERROR_FAULTM2       = 0x000080, //Error: Motor 2 Driver Fault (only on some models)
        ERROR_SPEED1        = 0x000100, //Error: Motor 1 Speed Error Limit
        ERROR_SPEED2        = 0x000200, //Error: Motor 2 Speed Error Limit
        ERROR_POS1          = 0x000400, //Error: Motor 1 Position Error Limit
        ERROR_POS2          = 0x000800, //Error: MOtor2 Position Error Limit
        WARN_OVERCURRENTM1  = 0x010000, //Warning: Motor 1 Current Limited
        WARN_OVERCURRENTM2  = 0x020000, //Warning: Motor 2 CUrrent Limited
        WARN_MBATHIGH       = 0x040000, //Warning: Main Battery Voltage High
        WARN_MBATLOW        = 0x080000, //Warning: Main Battery Low Voltage
        WARN_TEMP           = 0x100000, //Warning: Temperaure Sensor 1 >=85C
        WARN_TEMP2          = 0x200000, //Warning: Temperature Sensor 2 >=85C (available only on some models)
        WARN_S4             = 0x400000, //Warning: Motor 1 Home/Limit Signal
        WARN_S5             = 0x800000, //Warning: Motor 2 Home/Limit Signal
    };


    //! Set motor speed
    void setMotor1Speed(float throttle);
    void setMotor2Speed(float throttle);

    //! Get encoder values for motors
    uint32_t getMotor1Encoder();
    uint32_t getMotor2Encoder();

    void resetEncoders();

    //! Get motor speeds
    int getMotor1Speed();
    int getMotor2Speed();

    //! Get RoboClaw version string
    std::string getVersion();

    //! Get voltage of main battery
    float getBatteryVoltage();

    //! Get status of motor controller
    Status getStatus();

private:
    //------------------------------------------------------------------------
    // Enumerations
    //------------------------------------------------------------------------
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
        GETSTATUS = 90,
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
    public:
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

        bool check(SerialInterface &serialInterface)
        {
            // Read 1st CRCbyte
            uint8_t ccrc_1;
            if(!serialInterface.readByte(ccrc_1)) {
                throw std::runtime_error("Unable to read CRC");
            }

            // Read 2nd CRC byte
            uint8_t ccrc_2;
            if(!serialInterface.readByte(ccrc_2)) {
                throw std::runtime_error("Unable to read CRC");
            }

            // Check
            const uint16_t ccrc = (ccrc_1 << 8) | ccrc_2;
            return (ccrc == m_CRC);
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
    // End-stop of variadic expandion
    void write(CRC&)
    {
    }

    //! Write byte
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

    //! Write word
    template<typename ...Data>
    void write(CRC &crc,
               uint16_t word, Data... data)
    {
        // Write bytes of word followed by remainder
        // **NOTE** RoboClaw is big endian so SerialInterface::write wouldn't work
        write(crc, (uint8_t)(word >> 8),(uint8_t)word, data...);
    }

     //! Write dword
    template<typename ...Data>
    void write(CRC &crc,
               uint32_t dword, Data... data)
    {
        // Write bytes of dword followed by remainder
        // **NOTE** RoboClaw is big endian so SerialInterface::write wouldn't work
        write(crc, (uint8_t)(dword >> 24), (uint8_t)(dword >> 16), (uint8_t)(dword >> 8),(uint8_t)dword, data...);
    }


    //! Write command
    template<typename ...Data>
    void writeCommand(Command command, Data... data)
    {
        // Start CRC calculation
        CRC crc;

        // Write data
        write(crc, m_Address, static_cast<uint8_t>(command), data...);

        // Write CRC
        // **NOTE** RoboClaw is big endian so SerialInterface::write wouldn't work
        m_SerialInterface.writeByte(crc.get() >> 8);
        m_SerialInterface.writeByte(crc.get());

        // If byte is read successfully
        uint8_t response;
        if(m_SerialInterface.readByte(response) && response == 0xFF) {
            return;
        }
        else {
            throw std::runtime_error("Invalid response");
        }
    }

    //! End-stop of variadic expandion
    void read(CRC&)
    {
    }

    //! Read a byte
    template<typename ...Data>
    void read(CRC &crc,
              uint8_t *byte, Data... data)
    {
        // Read byte and update CRC
        if(m_SerialInterface.readByte(*byte)) {
            crc.update(*byte);
        }
        else {
            throw std::runtime_error("Unable to read byte");
        }

        // Read remainder
        read(crc, data...);
    }

    //! Read a word
    template<typename ...Data>
    void read(CRC &crc,
              uint16_t *word, Data... data)
    {
        // Read two bytes followed by remainder
        // **NOTE** RoboClaw is big endian so SerialInterface::read wouldn't work
        uint8_t bytes[2];
        read(crc, &bytes[0], &bytes[1], data...);

        // Re-assemble word
        *word = (bytes[0] << 8) | bytes[1];
    }

    //! Read a dword
    template<typename ...Data>
    void read(CRC &crc,
              uint32_t *dword, Data... data)
    {
        // Read four bytes followed by remainder
        // **NOTE** RoboClaw is big endian so SerialInterface::read wouldn't work
        uint8_t bytes[4];
        read(crc, &bytes[0], &bytes[1], &bytes[2], &bytes[3], data...);

        // Re-assemble dword
        *dword = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
    }

    // Perform a read command consisting of:
    // write address [1 byte]
    // write command[1 byte]
    // read variadic parameters
    // read CRC [2 bytes]
    template<typename ...Data>
    void readCommand(Command command, Data... data)
    {
        // Write address and command
        CRC crc;
        write(crc, m_Address, static_cast<uint8_t>(command));

        // Read data
        read(crc, data...);

        // Check CRC
        if(!crc.check(m_SerialInterface)) {
            throw std::runtime_error("CRC check failed");
        }
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    //! Serial interface for communicating with RoboClaw
    SerialInterface m_SerialInterface;

    //! Address of RoboClaw we're targetting
    const uint8_t m_Address;
};
}   // namespace BoBRobotics

#endif   // _WIN32
