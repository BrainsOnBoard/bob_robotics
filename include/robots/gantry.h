#pragma once

#include "os/windows_include.h"

// BoB robotics includes
#include "common/macros.h"
#include "common/pose.h"

// Third-party includes
#include "third_party/units.h"

// Gantry-specifc includes
#include "C:\Program Files\Advantech\Motion\PCI-1240\Examples\Include\Ads1240.h"

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace BoBRobotics {
namespace Robots {
using namespace std::literals;
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Robots::Gantry
//----------------------------------------------------------------------------
//! An interface for the robot gantry system at the University of Sussex
class Gantry
{
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    //! Open the PCI device and set drive parameters
    Gantry(BYTE boardId = 0)
      : m_BoardId(boardId)
    {
        // Try to open PCI device
        checkError(P1240MotDevOpen(m_BoardId), "Could not open PCI card");

        try {
            // Give error if emergency button pressed
            checkEmergencyButton();

            // Set params for line interpolation
            checkError(P1240SetStartSpeed(m_BoardId, IPO_Axis, 125), "Could not set start speed");
            checkError(P1240SetDrivingSpeed(m_BoardId, IPO_Axis, 1200), "Could not set driving speed");
            checkError(P1240SetAcceleration(m_BoardId, IPO_Axis, 400), "Could not set acceleration");
            checkError(P1240SetDeceleration(m_BoardId, IPO_Axis, 400, 0), "Could not set deceleration");
        } catch (std::exception &) {
            // In case of error, close PCI device and rethrow exception
            close();
            throw;
        }
    }

    ~Gantry() noexcept
    {
        // Stop the gantry moving
        stopMoving();

        // Close PCI device
        close();
    }

    /**!
     * \brief Returns the gantry to its home position, raising the gantry head first
     *
     * Home position is (0, 0, 0). The robot gantry will be raised before homing so that it does
     * not collide with objects in the arena. The gantry needs to be homed before use so that it
     * can reset its estimate of its position. This function blocks until the gantry is homed.
     */
    void raiseAndHome()
    {
        m_IsMovingLine = false;

        // Raise z-axis so we don't bump objects in arena
        checkError(P1240MotChgDV(m_BoardId, Z_Axis, 1200), "Could not set z-axis speed");
        checkError(P1240MotCmove(m_BoardId, Z_Axis, 0 /*up*/), "Could not move z-axis to limit");
        waitToStopMoving(Z_Axis);

        // Home along x- and y-axes
        home(XY_Axis);
        waitToStopMoving(XY_Axis);

        // Home z-axis
        home(Z_Axis);
        waitToStopMoving(Z_Axis);

        // Give error if emergency button pressed
        checkEmergencyButton();
    }

    /**!
     * \brief Returns the gantry to its home position
     *
     * Home position is (0, 0, 0). The gantry needs to be homed before use so that it
     * can reset its estimate of its position. This function does not block.
     */
    void home(BYTE axis = XYZ_Axis) const
    {
        checkError(P1240MotHome(m_BoardId, axis), "Could not home axis");
    }

    //! Check if either of the emergency buttons are pressed down
    bool isEmergencyButtonPressed() const
    {
        DWORD ret;
        checkError(P1240MotRdReg(m_BoardId, 1, RR2, &ret), "Could not read emergency button flag");
        return ret & 32;
    }

    //! Get the current position of the gantry in the arena
    Vector3<millimeter_t> getPosition() const
    {
        // Request position from card
        std::array<LONG, 3> pulses;
        checkError(P1240GetTheorecticalRegister(m_BoardId, X_Axis, &pulses[0]), "Could not get x position");
        checkError(P1240GetTheorecticalRegister(m_BoardId, Y_Axis, &pulses[1]), "Could not get y position");
        checkError(P1240GetTheorecticalRegister(m_BoardId, Z_Axis, &pulses[2]), "Could not get z position");

        // Convert to LengthUnit
        return pulsesToUnit<millimeter_t>(pulses);
    }

    //! Get the gantry's current velocity
    std::array<meters_per_second_t, 3> getVelocity()
    {
        std::array<DWORD, 3> pulseRate;

        m_IsMovingLine = m_IsMovingLine && isMoving();
        if (m_IsMovingLine) {
            /*
             * The driver seems to behave oddly when trying to read the velocity in line mode: a value is only given
             * for the first axis, which apparently corresponds to the "interpolation" axis, not the x-axis (the other
             * values are zero). This value seems to vary sensibly over the course of movements, but it's not clear
             * how to use this value to get x, y, z values.
             */
            throw std::runtime_error("Getting velocity when moving in line mode is not currently supported");
        } else {
            checkError(P1240MotRdMultiReg(m_BoardId, XYZ_Axis, CurV, &pulseRate[0], &pulseRate[1], &pulseRate[2], nullptr), "Error reading velocity");
        }

        using InitialUnit = units::unit_t<units::compound_unit<units::length::millimeter, units::inverse<units::time::second>>>;
        const auto velocity = pulsesToUnit<meters_per_second_t, InitialUnit, DWORD>(pulseRate);
        return velocity;
    }

    /**!
     * \brief Set the position of the gantry in the arena
     *
     * This function does not block.
     */
    void setPosition(millimeter_t x, millimeter_t y, millimeter_t z)
    {
        // Check the desired position is within the gantry's limits
        BOB_ASSERT(x >= 0_mm && x <= Limits[0]);
        BOB_ASSERT(y >= 0_mm && y <= Limits[1]);
        BOB_ASSERT(z >= 0_mm && z <= Limits[2]);

        m_IsMovingLine = true;
        const std::array<LONG, 3> pos = { (LONG) round(x.value() * PulsesPerMillimetre[0]),
                                    (LONG) round(y.value() * PulsesPerMillimetre[1]),
                                    (LONG) round(z.value() * PulsesPerMillimetre[2]) };
        checkError(P1240MotLine(m_BoardId, XYZ_Axis, TRUE, pos[0], pos[1], pos[2], 0), "Could not move gantry");
    }

    //! Stop the gantry moving, optionally specifying a specific axis
    void stopMoving(BYTE axis = XYZ_Axis) const noexcept
    {
        P1240MotStop(m_BoardId, axis, axis * SlowStop);
    }

    //! Check if the gantry is moving
    bool isMoving(BYTE axis = XYZ_Axis) const
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
        }

        throwError(res, "Error while reading axis busy state");
    }

    //! Wait until the gantry has stopped moving
    void waitToStopMoving(BYTE axis = XYZ_Axis) const
    {
        // Repeatedly poll card to check whether gantry is moving
        while (isMoving(axis)) {
            std::this_thread::sleep_for(10ms);
        }

        /*
         * If the emergency button is pressed, the gantry will have
         * stopped moving, so we should check the button so we can
         * terminate gracefully.
         */
        checkEmergencyButton();
    }

private:
    BYTE m_BoardId;
    bool m_IsMovingLine = false;

    /*
     * These values are for converting from a number of motor pulses in the x, y
     * and z axes to a number of millimetres. I took them directly from Chris
     * Johnson's Matlab code and I assume he just measured them empirically.
     * They seem pretty accurate. -- AD
     */
    static constexpr std::array<double, 3> PulsesPerMillimetre = { 7.49625, 8.19672, 13.15789 };

    // These are the gantry's upper x, y and z limits (i.e. the size of the "arena")
    static constexpr std::array<millimeter_t, 3> Limits = { 2996_mm, 1793_mm, 1203_mm };

    void close() const noexcept
    {
        // Close PCI device
        P1240MotDevClose(m_BoardId);
    }

    inline void checkEmergencyButton() const
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

    template<class UnitType, class InitialUnit = millimeter_t, class T>
    static auto pulsesToUnit(const std::array<T, 3> &pulses)
    {
        std::array<UnitType, 3> unitArray;
        std::transform(pulses.begin(), pulses.end(), PulsesPerMillimetre.begin(), unitArray.begin(), [](T pulse, double permm) {
            return units::make_unit<InitialUnit>((double) pulse / permm);
        });
        return unitArray;
    }
}; // Gantry
} // Robots
} // BoBRobotics
