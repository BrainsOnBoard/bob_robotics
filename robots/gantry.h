#pragma once

#include "os/windows_include.h"

// Standard C++ includes
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

// Gantry-specifc includes
#include "C:\Program Files\Advantech\Motion\PCI-1240\Examples\Include\Ads1240.h"

// BoB robotics includes
#include "common/pose.h"

// Third-party includes
#include "third_party/units.h"

using namespace units::literals;

namespace BoBRobotics {
namespace Robots {
using namespace std::literals;
using namespace units::math;
using namespace units::length;

class Gantry
{
public:
    Gantry(BYTE boardId = 0)
      : m_BoardId(0)
    {
        // Try to open PCI device
        checkError(P1240MotDevOpen(m_BoardId), "Could not open PCI card");

        // Give error if emergency button pressed
        checkEmergencyButton();
    }

    ~Gantry()
    {
        stopMoving();

        // Close PCI device
        P1240MotDevClose(m_BoardId);
    }

    void home()
    {
        // Raise z-axis so we don't bump objects in arena
        checkError(P1240MotChgDV(m_BoardId, Z_Axis, 1000), "Could not set z-axis speed");
        checkError(P1240MotCmove(m_BoardId, Z_Axis, 0 /*up*/), "Could not move z-axis to limit");
        waitToStopMoving(Z_Axis);

        // Home along x- and y-axes
        checkError(P1240MotHome(m_BoardId, XY_Axis), "Could not home x- and y- axes");
        waitToStopMoving(XY_Axis);

        // Home z-axis
        checkError(P1240MotHome(m_BoardId, Z_Axis), "Could not home z-axis");
        waitToStopMoving(Z_Axis);

        // Give error if emergency button pressed
        checkEmergencyButton();
    }

    bool isEmergencyButtonPressed()
    {
        DWORD ret;
        checkError(P1240MotRdReg(m_BoardId, 1, RR2, &ret), "Could not read emergency button flag");
        return ret & 32;
    }

    template<class LengthUnit = millimeter_t>
    Vector3<LengthUnit> getPosition()
    {
        Vector3<LengthUnit> pos;

        // Request position from card
        LONG val;
        checkError(P1240GetTheorecticalRegister(m_BoardId, X_Axis, &val), "Could not get x position");
        pos[0] = (double) val / PulsesPerMillimetreX;
        checkError(P1240GetTheorecticalRegister(m_BoardId, Y_Axis, &val), "Could not get y position");
        pos[1] = (double) val / PulsesPerMillimetreY;
        checkError(P1240GetTheorecticalRegister(m_BoardId, Z_Axis, &val), "Could not get z position");
        pos[2] = (double) val / PulsesPerMillimetreZ;

        return pos;
    }

    void setPosition(millimeter_t x, millimeter_t y, millimeter_t z)
    {
        // Set params for line interpolation
        checkError(P1240SetDrivingSpeed(m_BoardId, 0, 1000), "Could not set driving speed");
        checkError(P1240SetStartSpeed(m_BoardId, 0, 125), "Could not set start speed");

        const LONG posX = (LONG) round(x * PulsesPerMillimetreX);
        const LONG posY = (LONG) round(y * PulsesPerMillimetreY);
        const LONG posZ = (LONG) round(z * PulsesPerMillimetreZ);
        checkError(P1240MotLine(m_BoardId, XYZ_Axis, XYZ_Axis, posX, posY, posZ, 0), "Could not move gantry");
    }

    void stopMoving(BYTE axis = XYZ_Axis) noexcept
    {
        // To stop moving immediately, cf. gradually, set the 3rd arg to zero
        P1240MotStop(m_BoardId, axis, axis);
    }

    bool isMoving(BYTE axis = XYZ_Axis)
    {
        // Indicate whether specified axis/axes busy
        LRESULT res = P1240MotAxisBusy(m_BoardId, axis);
        switch (res) {
        case AxisDrvBusy:
        case AxisHomeBusy:
        case AxisExtBusy:
            return true;
        case ERROR_SUCCESS:
            return false;
        default:
            throwError(res, "Error while reading axis busy state");
        }
    }

    void waitToStopMoving(BYTE axis = XYZ_Axis)
    {
        // Repeatedly poll card to check whether gantry is moving
        while (isMoving(axis)) {
            std::this_thread::sleep_for(10ms);
        }

        checkEmergencyButton();
    }

private:
    BYTE m_BoardId;
    static constexpr auto PulsesPerMillimetreX = 7.49625 / 1_mm;
    static constexpr auto PulsesPerMillimetreY = 8.19672 / 1_mm;
    static constexpr auto PulsesPerMillimetreZ = 13.15789 / 1_mm;

    inline void checkEmergencyButton()
    {
        if (isEmergencyButtonPressed()) {
            throw std::runtime_error("Gantry error: Emergency button is pressed");
        }
    }

    inline static void checkError(LRESULT res, const std::string &msg)
    {
        if (res) {
            throwError(res, msg);
        }
    }

    template<class T>
    inline static void throwError(T res, const std::string &msg)
    {
        std::stringstream sstream;
        sstream << "Gantry error (0x" << std::hex << res << "): " << msg;
        throw std::runtime_error(sstream.str());
    }
}; // Gantry
} // Robots
} // BoBRobotics
