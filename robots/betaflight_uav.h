#pragma once

// BoB robotics includes
#include "../common/not_implemented.h"
#include "uav.h"

// Third-party includes
#include <msp/FlightController.hpp>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <iostream>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Robots {

class BetaflightUAV : public UAV
{
private:
    struct MyIdent : public msp::Request
    {
        msp::ID id() const
        {
            return msp::ID::MSP_STATUS_EX;
        }

        msp::ByteVector raw_data;

        void decode(const msp::ByteVector &data)
        {
            raw_data = data;
        }
    };

    struct Analog : public msp::Request
    {
        msp::ID id() const
        {
            return msp::ID::MSP_ANALOG;
        }

        float vbat;          // Volt
        float powerMeterSum; // Ah
        size_t rssi;         // Received Signal Strength Indication [0; 1023]
        float amperage;      // Ampere

        void decode(const std::vector<uint8_t> &data)
        {
            vbat = data[0] / 10.0f;
            //powerMeterSum = data[1]/1000.0f;
            //rssi          = data[3];
            amperage = data[5] / 10.0f;
        }
    };

public:
    BetaflightUAV(const std::string &device, int baud)
      : m_Fcu(device, baud)
    {
        m_Fcu.initialise();
    }

    void subscribe()
    {
        m_Fcu.subscribe(&BetaflightUAV::onIdent, this, 0.1);
        m_Fcu.subscribe(&BetaflightUAV::onAnalog, this, 0.5);
    }

    void onIdent(const MyIdent &ident)
    {
        m_CurrentArmFlags.clear();

        const int flagData = *((int *) (&ident.raw_data[17]));
        for (size_t i = 0; i < ArmFlags.size(); i++) {
            if (flagData & (int) (1 << i)) {
                m_CurrentArmFlags.append(ArmFlags[i]);
                m_CurrentArmFlags.append(std::string(" (Error number ") + std::to_string(i) + std::string("), "));
            }
        }
    }

    void onAnalog(const Analog &anog)
    {
        //std::cout << "Battery voltage level: " << anog.vbat << " V." << std::endl;
        m_CurrentVoltage = anog.vbat;
        //std::cout << "Power Meter Summery: " << anog.powerMeterSum << std::endl;
        //std::cout << "Received Signal Strength Indication: " << anog.rssi << std::endl;
        //std::cout << "Current level: " << anog.amperage << " mA." << std::endl;
        m_CurrentAmpDraw = anog.amperage;
    }

    BOB_NOT_IMPLEMENTED(virtual void takeOff() override)
    BOB_NOT_IMPLEMENTED(virtual void land() override)

    virtual void setRoll(float right) override
    {
        right = std::min(1.0f, std::max(-1.0f, right));
        m_RCValues[0] = 1500 + right * m_ControlScale;
    }

    virtual void setPitch(float pitch) override
    {
        pitch = std::min(1.0f, std::max(-1.0f, pitch));
        m_RCValues[1] = 1500 + pitch * m_ControlScale;
    }

    virtual void setVerticalSpeed(float up) override
    {
        up = std::min(1.0f, std::max(-1.0f, up));
        m_RCValues[2] = 1500 + up * m_ThrottleScale;
    }

    virtual void setYawSpeed(float right) override
    {
        right = std::min(1.0f, std::max(-1.0f, right));
        m_RCValues[3] = 1500 + right * m_ControlScale;
    }

    const std::string &getArmStateAsString() const
    {
        return m_CurrentArmFlags;
    }

    float getVoltage() const
    {
        return m_CurrentVoltage;
    }

    float getAmpDraw() const
    {
        return m_CurrentAmpDraw;
    }

    // accessors for m_ControlScale
    void setControlScale(float scale)
    {
        this->m_ControlScale = scale;
    }

    float getControlScale()
    {
        return this->m_ControlScale;
    }

    void sendCommands()
    {
        std::vector<uint16_t> auxs;
        m_Fcu.setRc(m_RCValues[0], m_RCValues[1], m_RCValues[3], m_RCValues[2], m_RCValues[4], m_RCValues[5], m_RCValues[6], m_RCValues[7], auxs);
    }

    void arm()
    {
        // hold throttle low to arm
        //m_RCValues[2] = 1100;
        m_RCValues[5] = 2000;
    }

    void disarm()
    {
        m_RCValues[5] = 1000;
        // reset other values
        m_RCValues[0] = 1500;
        m_RCValues[1] = 1500;
        m_RCValues[2] = 1040;
        m_RCValues[3] = 1500;
    }

private:
    fcu::FlightController m_Fcu;
    uint16_t m_RCValues[8] = { 1500, 1500, 1040, 1500, 2000, 1000, 1500, 1500 };
    std::string m_CurrentArmFlags;
    float m_ControlScale = 100.0f;
    float m_ThrottleScale = 150.0f;
    float m_CurrentVoltage = 0.0;
    float m_CurrentAmpDraw = 0.0;

    static constexpr std::array<const char *, 18> ArmFlags{
        "Gyro not detected",
        "Fail safe active",
        "RX disconnected",
        "RX connected, Arm switch on, turn off Arming",
        "Failsafe switch active",
        "Runaway takeoff prevention",
        "Throttle too high",
        "Craft not level",
        "Arming too soon after power up",
        "Prearm not active",
        "Sytem load too high",
        "Sensor calibration still ongoing",
        "CLI active",
        "Config menu active - over OSD other display",
        "OSD menu active",
        "A Black Sheep Telemetry device (TBS Core Pro for example) disarmed and is preventing arming",
        "MSP connection is active, probably via Betaflight Configurator",
        "Arm switch is in an unsafe position"
    };
};

/*
 * C++ requires a definition and declaration for static members, but we'll get
 * linking errors if we declare it twice, so we use this kludge for if we're
 * linking multiple objects.
 */
#ifndef NO_HEADER_DEFINITIONS
constexpr std::array<const char *, 18> BetaflightUAV::ArmFlags;
#endif

} // Robots
} // BoBRobotics
