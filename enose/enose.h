#pragma once

// Serial port includes
#include <SerialPort.h>

// BoB robotics includes
#include "common/fsm.h"

//----------------------------------------------------------------------------
// BoBRobotics::ENoseState
//----------------------------------------------------------------------------
namespace BoBRobotics
{
enum class ENoseState
{
    Invalid,
    Waiting,
    ReadingHeader,
    ReadingData,
};

//----------------------------------------------------------------------------
// StateHandler
//----------------------------------------------------------------------------
class ENose : public FSM<ENoseState>::StateHandler
{
public:
    ENose(const char *device) : m_FSM(this, ENoseState::Invalid), m_SerialPort(device), m_HasData(false)
    {
        // Open serial port
        m_SerialPort.Open(SerialPort::BAUD_500000);

        // Wait for data
        m_FSM.transition(ENoseState::Waiting);
    }

    //------------------------------------------------------------------------
    // Statehandler virtuals
    //------------------------------------------------------------------------
    virtual bool handleEvent(ENoseState state, Event event) override
    {
        if(state == ENoseState::Waiting) {
            if(event == Event::Enter) {
                std::cout << "Waiting for device identification" << std::endl;
            }
            else if(event == Event::Update) {
                const std::string line = readLine();
                if(line == "#!ENOSE-V3.0-analog") {
                    // Send byte to start transmission
                    m_SerialPort.WriteByte('*');

                    m_FSM.transition(ENoseState::ReadingHeader);
                }
                else {
                    std::cerr << "Unexpected line received - maybe restart and try again" << std::endl;
                    return false;
                }
            }
        }
        else if(state == ENoseState::ReadingHeader) {
            if(event == Event::Enter) {
                std::cout << "Reading header" << std::endl;
            }
            if(event == Event::Update) {
                const std::string line = readLine();
                if(line[0] == '@') {
                }
                else if(line[0] == '#') {
                    if(line.substr(0, 9) == "#columns:") {
                        if(line == "#columns: S0:<u4 S1:<u4 S2:<u4 S3:<u4 TEMP:<u2 RH:<u2") {
                            m_FSM.transition(ENoseState::ReadingData);
                        }
                        else {
                            std::cerr << "Unsupported streaming format" << std::endl;
                            return false;
                        }
                    }
                }
                else {
                    std::cerr << "Unexpected header line (" << line << ") received - maybe restart and try again" << std::endl;
                    return false;
                }
            }
        }
        else if(state == ENoseState::ReadingData) {
            if(event == Event::Enter) {
                std::cout << "Reading data" << std::endl;
            }
            else if(event == Event::Update) {
                // Read first byte
                const char firstByte = m_SerialPort.ReadByte(5000);

                if(firstByte == '#') {
                    discardLine();
                }
                else if(firstByte == 'b') {
                    // Set flag
                    m_HasData = true;

                    // Read gas bytes
                    for(size_t i = 0; i < (4 * sizeof(uint32_t)); i++) {
                        m_Gas.Bytes[i] = m_SerialPort.ReadByte(5000);
                    }

                    // Read temperature bytes
                    for(size_t i = 0; i < (2 * sizeof(uint16_t)); i++) {
                        m_Temperature.Bytes[i] = m_SerialPort.ReadByte(5000);
                    }

                    // Discard remainder of line
                    discardLine();
                }
                else {
                    std::cerr << "Unexpected data line received - maybe restart and try again" << std::endl;
                    return false;
                }
            }
        }
        else {
            throw std::runtime_error("Invalid state");
        }

        return true;
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool update()
    {
        return m_FSM.update();
    }

    const auto &getGas() const{ return m_Gas.Words; }
    const auto &getTemperature() const{ return m_Temperature.Shorts; }

    bool hasData() const{ return m_HasData; }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    std::string readLine()
    {
        std::string line;
        try {
            // Read line
            line = m_SerialPort.ReadLine(5000) ;
        }
        catch (const SerialPort::ReadTimeout&) {
            throw std::runtime_error("The Read() call timed out waiting for additional data");
        }

        // Remove line end characters (\r\n I'm guessing)
        line.pop_back();
        line.pop_back();
        return line;
    }

    void discardLine()
    {
        // Read bytes until end of line is hit
        char byte;
        do {
            byte = m_SerialPort.ReadByte(5000);
        } while(byte != '\n');
    }

    //------------------------------------------------------------------------
    // Unions
    //------------------------------------------------------------------------
    // Union for converting (little-endian) bytes to 4 gas sensor channels
    union Gas
    {
        uint32_t Words[4];
        char Bytes[16];
    };

    // Union for converting (little-endian) bytes to 2 temperature sensor channels
    union Temperature
    {
        uint16_t Shorts[2];
        char Bytes[4];
    };

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    FSM<ENoseState> m_FSM;

    SerialPort m_SerialPort;

    Gas m_Gas;
    Temperature m_Temperature;

    bool m_HasData;
};
}
