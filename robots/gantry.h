#pragma once

#include "os/windows_include.h"

// Standard C++ includes
#include <algorithm>
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
using namespace units;
using namespace units::math;
using namespace units::length;
using namespace units::time;
using namespace units::velocity;

//----------------------------------------------------------------------------
// BoBRobotics::Robots::Gantry
//----------------------------------------------------------------------------
//! An interface for the robot gantry system at the University of Sussex
class Gantry
{
public:
	//! Open the PCI device and set drive parameters
    Gantry(BYTE boardId = 0)
      : m_BoardId(0)
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

    ~Gantry()
    {
        // Stop the gantry moving
        stopMoving();

        // Close PCI device
        close();
    }

	/**!
	 * \brief Returns the gantry to its home position
	 *
	 * Home position is (0, 0, 0). The robot gantry will be raised before homing so that it does
	 * not collide with objects in the arena. The gantry needs to be homed before use so that it
	 * can reset its estimate of its position. This function blocks until the gantry is homed.
	 */
    void home()
    {
        m_IsMovingLine = false;

        // Raise z-axis so we don't bump objects in arena
        checkError(P1240MotChgDV(m_BoardId, Z_Axis, 1200), "Could not set z-axis speed");
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

	//! Check if either of the emergency buttons are pressed down
    bool isEmergencyButtonPressed()
    {
        DWORD ret;
        checkError(P1240MotRdReg(m_BoardId, 1, RR2, &ret), "Could not read emergency button flag");
        return ret & 32;
    }

	//! Get the current position of the gantry in the arena
    template<class LengthUnit = millimeter_t>
    Vector3<LengthUnit> getPosition()
    {
        // Request position from card
        Vector3<LONG> pulses;
        checkError(P1240GetTheorecticalRegister(m_BoardId, X_Axis, &pulses[0]), "Could not get x position");
        checkError(P1240GetTheorecticalRegister(m_BoardId, Y_Axis, &pulses[1]), "Could not get y position");
        checkError(P1240GetTheorecticalRegister(m_BoardId, Z_Axis, &pulses[2]), "Could not get z position");

        // Convert to LengthUnit
        return pulsesToUnit<LengthUnit>(pulses);
    }

	//! Get the gantry's current velocity
    template<class VelocityUnit = meters_per_second_t>
    Vector3<VelocityUnit> getVelocity()
    {
        Vector3<DWORD> pulseRate;

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

        using InitialUnit = unit_t<compound_unit<millimeter, inverse<second>>>;
        const auto velocity = pulsesToUnit<VelocityUnit, InitialUnit, DWORD>(pulseRate);
        return velocity;
    }

	/**!
	 * \brief Set the position of the gantry in the arena
	 *
	 * This function does not block.
	 */
    void setPosition(millimeter_t x, millimeter_t y, millimeter_t z)
    {
        m_IsMovingLine = true;
        const Vector3<LONG> pos = { (LONG) round(x.value() * PulsesPerMillimetre[0]),
                                    (LONG) round(y.value() * PulsesPerMillimetre[1]),
                                    (LONG) round(z.value() * PulsesPerMillimetre[2]) };
        checkError(P1240MotLine(m_BoardId, XYZ_Axis, TRUE, pos[0], pos[1], pos[2], 0), "Could not move gantry");
    }

	//! Stop the gantry moving, optionally specifying a specific axis
    void stopMoving(BYTE axis = XYZ_Axis) noexcept
    {
        P1240MotStop(m_BoardId, axis, axis * SlowStop);
    }

	//! Check if the gantry is moving
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

	//! Wait until the gantry has stopped moving
    void waitToStopMoving(BYTE axis = XYZ_Axis)
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
    static constexpr std::array<double, 3> PulsesPerMillimetre = { 7.49625, 8.19672, 13.15789 };

    void close() noexcept
    {
        // Close PCI device
        P1240MotDevClose(m_BoardId);
    }

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

    template<class UnitType, class InitialUnit = millimeter_t, class T>
    static auto pulsesToUnit(const Vector3<T> &pulses)
    {
        Vector3<UnitType> unitArray;
        std::transform(pulses.begin(), pulses.end(), PulsesPerMillimetre.begin(), unitArray.begin(), [](T pulse, double permm) {
            return units::make_unit<InitialUnit>((double) pulse / permm);
        });
        return unitArray;
    }
}; // Gantry
} // Robots
} // BoBRobotics
