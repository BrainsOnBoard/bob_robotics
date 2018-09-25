#pragma once

#include "os/windows_include.h"

// Standard C++ includes
#include <chrono>
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

class GantryError : public std::runtime_error
{
public:
    GantryError(const std::string &msg, LRESULT err)
      : std::runtime_error(msg + " (error no: "s + std::to_string(err) + ")"s)
    {}
};

class Gantry
{
public:
    enum Axis : BYTE
    {
        XAxis = 1,
        YAxis = 2,
        ZAxis = 4,
        AllAxes = XAxis | YAxis | ZAxis
    };

    Gantry()
    {
        // Try to open PCI device
        CheckError(P1240MotDevOpen(BoardId), "Could not open PCI card");

        if (isEmergencyButtonPressed()) {
            throw std::runtime_error("Emergency button is pressed");
        }
    }

    ~Gantry()
    {
        // Close PCI device
        P1240MotDevClose(BoardId);
    }

    void home()
    {
        // Raise z-axis so we don't bump objects in arena
        CheckError(P1240MotChgDV(BoardId, ZAxis, 1000), "Could not set z-axis motor speed");
        CheckError(P1240MotCmove(BoardId, ZAxis, 0), "Could not move z-axis to limit");
        waitToStopMoving(ZAxis);

        // Home along x- and y-axes
        CheckError(P1240MotHome(BoardId, XAxis | YAxis), "Could not home x- and y- axes");
        waitToStopMoving(XAxis | YAxis);

        // Home z-axis
        CheckError(P1240MotHome(BoardId, ZAxis), "Could not home z-axis");
        waitToStopMoving(ZAxis);
    }

    bool isEmergencyButtonPressed()
    {
        DWORD ret;
        CheckError(P1240MotRdReg(BoardId, 1, RR2, &ret), "Could not read emergency button flag");
        return ret & 32;
    }

    Vector3<millimeter_t> getPosition()
    {
        Vector3<millimeter_t> pos;

        // Request position from card
        LONG val;
        CheckError(P1240GetTheorecticalRegister(BoardId, XAxis, &val), "Could not get x position");
        pos[0] = (double) val / PulsesPerMillimetreX;
        CheckError(P1240GetTheorecticalRegister(BoardId, YAxis, &val), "Could not get y position");
        pos[1] = (double) val / PulsesPerMillimetreY;
        CheckError(P1240GetTheorecticalRegister(BoardId, ZAxis, &val), "Could not get z position");
        pos[2] = (double) val / PulsesPerMillimetreZ;

        return pos;
    }

    void setPosition(millimeter_t x, millimeter_t y, millimeter_t z)
    {
        if (isEmergencyButtonPressed()) {
            throw std::runtime_error("Emergency button is pressed");
        }

        P1240SetDrivingSpeed(BoardId, 0, 1000);
        P1240SetStartSpeed(BoardId, 0, 125);

        const LONG posX = (LONG) round(x * PulsesPerMillimetreX);
        const LONG posY = (LONG) round(y * PulsesPerMillimetreY);
        const LONG posZ = (LONG) round(z * PulsesPerMillimetreZ);
        CheckError(P1240MotLine(BoardId, AllAxes, AllAxes, posX, posY, posZ, 0), "Could not move gantry");
    }

    bool isMoving(BYTE axis = AllAxes)
    {
        // Indicate whether specified axis/axes busy
        return P1240MotAxisBusy(BoardId, axis);
    }

    void waitToStopMoving(BYTE axis = AllAxes)
    {
        // Repeatedly poll card to check whether gantry is moving
        while (isMoving(axis)) {
            std::this_thread::sleep_for(25ms);
        }
    }

private:
    static constexpr BYTE BoardId = 0;
    static constexpr auto PulsesPerMillimetreX = 7.49625 / 1_mm;
    static constexpr auto PulsesPerMillimetreY = 8.19672 / 1_mm;
    static constexpr auto PulsesPerMillimetreZ = 13.15789 / 1_mm;

    static void CheckError(LRESULT err, const std::string &msg)
    {
        if (err) {
            throw GantryError(msg, err);
        }
    }
}; // Gantry
} // Robots
} // BoBRobotics
