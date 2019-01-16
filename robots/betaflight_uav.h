#pragma once

// BoB robotics includes
#include "../common/not_implemented.h"
#include "uav.h"

// Third-party includes
#include "../third_party/units.h"
#include "msp/FlightController.hpp"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <bitset>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Robots {

class BetaflightUAV : public UAV
{
    using volt_t = units::voltage::volt_t;
    using ampere_t = units::current::ampere_t;
    using ErrorFlags = std::bitset<18>;

public:
    BetaflightUAV(const std::string &device, const int baud)
      : m_Fcu(device, baud)
    {
        m_Fcu.initialise();
    }

    void subscribe()
    {
        m_Fcu.subscribe(&BetaflightUAV::onIdent, this, 0.1);
        m_Fcu.subscribe(&BetaflightUAV::onAnalog, this, 0.5);
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

    std::string getArmState()
    {
        std::lock_guard<std::mutex> guard(m_ErrorsMutex);
        std::stringstream ss;
        for (size_t i = 0; i < ArmFlags.size(); i++) {
            if (m_Errors[i]) {
                if (!ss.str().empty()) {
                    ss << ", ";
                }
                ss << ArmFlags[i] << " (Error number " << i << ")";
            }
        }

        return ss.str();
    }

    volt_t getVoltage() const
    {
        return m_CurrentVoltage;
    }

    ampere_t getAmpDraw() const
    {
        return m_CurrentAmpDraw;
    }

    // accessors for m_ControlScale
    void setControlScale(float scale)
    {
        m_ControlScale = scale;
    }

    float getControlScale() const
    {
        return m_ControlScale;
    }

    void sendCommands()
    {
        m_Fcu.setRc(m_RCValues[0], m_RCValues[1], m_RCValues[3], m_RCValues[2],
                    m_RCValues[4], m_RCValues[5], m_RCValues[6], m_RCValues[7], {});
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
    ErrorFlags m_Errors;
    volt_t m_CurrentVoltage;
    ampere_t m_CurrentAmpDraw;
    std::mutex m_ErrorsMutex;
    float m_ControlScale = 100.0f;
    float m_ThrottleScale = 150.0f;

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

    struct Ident : public msp::Request
    {
        ErrorFlags errors;

        void decode(const msp::ByteVector &data)
        {
            errors = ErrorFlags(*reinterpret_cast<const int *>(data.data() + 17));
        }

        virtual msp::ID id() const override
        {
            return msp::ID::MSP_STATUS_EX;
        }
    };

    struct Analog : public msp::Request
    {
        volt_t voltage;
        ampere_t current;

        void decode(const msp::ByteVector &data)
        {
            voltage = volt_t{ data[0] / 10.0 };
            current = ampere_t{ data[5] / 10.0 };
            //powerMeterSum = data[1]/1000.0f;
            //rssi          = data[3];
        }

        virtual msp::ID id() const override
        {
            return msp::ID::MSP_ANALOG;
        }
    };

    void onIdent(const Ident &ident)
    {
        std::lock_guard<std::mutex> guard(m_ErrorsMutex);
        m_Errors = std::move(ident.errors);
    }

    void onAnalog(const Analog &anog)
    {
        m_CurrentVoltage = anog.voltage;
        m_CurrentAmpDraw = anog.current;
    }
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
