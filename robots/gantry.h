#pragma once

// Standard C++ includes
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

// Gantry-specifc includes
#include "../os/windows_include.h"
#include "C:\Program Files\Advantech\Motion\PCI-1240\Examples\Include\Ads1240.h"

namespace BoBRobotics {
namespace Robots {
using namespace std::literals;

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

    bool isMoving(BYTE axis = AllAxes)
    {
        // Indicate whether specified axis/axes busy
        return P1240MotAxisBusy(BoardId, axis);
    }

    void waitToStopMoving(BYTE axis = AllAxes)
    {
        // Repeatedly poll card to check whether gantry is moving
        while (isMoving()) {
            std::this_thread::sleep_for(25ms);
        }
    }

private:
    static constexpr BYTE BoardId = 0;

    static void CheckError(LRESULT err, const std::string &msg)
    {
        if (err) {
            throw GantryError(msg, err);
        }
    }
};
}
}