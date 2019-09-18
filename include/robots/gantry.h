#pragma once

#include "os/windows_include.h"

// BoB robotics includes
#include "common/logging.h"
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

    //! Get the gantry's current velocity {x,y,z,xyz-interpolated,xy-interpolated}
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
            DWORD h_xy_z = 0;
            checkError(P1240MotRdMultiReg(m_BoardId, XYZ_Axis, CurV, &h_xy_z, &pulseRate[1], &pulseRate[2], nullptr), "Error reading velocity");
            std::array<double, 2> angles = calcMoveAngles();
            pulseRate[2] = (DWORD) round((double) h_xy_z * sin(angles[1]));
            DWORD h_x_y = (DWORD) round((double) h_xy_z * cos(angles[1]));
            pulseRate[0] = (DWORD) round((double) h_x_y * sin(angles[0]));
            pulseRate[1] = (DWORD) round((double) h_x_y * cos(angles[0]));
            pulseRate[2] = h_xy_z;
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

        LOGD << "Moving to position: " << x << ", " << y << ", " << z;

        m_IsMovingLine = true;
        const std::array<LONG, 3> pos = { (LONG) round(x.value() * PulsesPerMillimetre[0]),
                                          (LONG) round(y.value() * PulsesPerMillimetre[1]),
                                          (LONG) round(z.value() * PulsesPerMillimetre[2]) };
        checkError(P1240MotLine(m_BoardId, XYZ_Axis, TRUE, pos[0], pos[1], pos[2], 0), "Could not move gantry");

        LOGD << "Starting move"

                /*
         * Save the target position of the line move to split interpolation
         * velocity into axis velocity
         */
                LastLineTarget[0] = x;
        LastLineTarget[1] = y;
        LastLineTarget[2] = z;

        refreshSpeeds();
    }

    /**!
     * \brief Calculates the trajectory of the gantry based on the angle along
     *        the xy-plane ([0]) and the angle between the xy-plane and the
     *        z-axis ([1])
     */
    std::array<double, 2> calcMoveAngles()
    {
        std::array<double, 2> Traj = { 0, 0 };
        std::array<millimeter_t, 3> currentPosition = getPosition();
        /*
              |    \
              |     \
            x |   \  h_x_y
              |       \
              |_____\ <-- theta_x_y
                y

              |    \
              |     \
            z |   \   h_xy_z
              |       \
              |_____\ <-- theta_xy_z
                h_x_y
            */
        double x = abs((double) LastLineTarget[0].value() - (double) currentPosition[0].value());
        double y = abs((double) LastLineTarget[1].value() - (double) currentPosition[1].value());
        double z = abs((double) LastLineTarget[2].value() - (double) currentPosition[2].value());

        //NOTE: when the num or denum in the atan function are 0 it return inf, not as it should 0 or pi/2 rads
        double h_x_y = 0;
        if (y == 0) {
            h_x_y = x;
        } else if (x == 0) {
            Traj[0] = 1.5707963267948966192313216916398;
            h_x_y = y;
        } else {
            Traj[0] = atan(x / y);
            h_x_y = x / sin(Traj[0]);
        }

        if (h_x_y == 0) {
            Traj[1] = 1.5707963267948966192313216916398;
        } else if (z != 0) {
            Traj[1] = atan(z / h_x_y);
        }

        return Traj;
    }

    /**!
     * \brief Set speed when moving the axis individually (None line movement speed)
     *
     * NOTE: The gantry seems to reset speed (90% of the time) when starting a
     * new move therefore the speed has to be adjusted after issuing the
     * move-cmd.
     */
    void setSpeed(meters_per_second_t x_speed, meters_per_second_t y_speed, meters_per_second_t z_speed)
    {
        DWORD PulseRate = (DWORD) round(x_speed.value() / MetersPerSecondPerPulseRate[1]);
        TargetSpeeds[0] = PulseRate;
        PulseRate = (DWORD) round(y_speed.value() / MetersPerSecondPerPulseRate[1]);
        TargetSpeeds[1] = PulseRate;
        PulseRate = (DWORD) round(z_speed.value() / MetersPerSecondPerPulseRate[2]);
        TargetSpeeds[2] = PulseRate;
        refreshSpeeds();
    }

    //! Set interpolation speed when moving the gantry using line movement
    void setSpeed(meters_per_second_t velocity)
    {
        interpolationVelocity = velocity;
        refreshSpeeds();
    }

    /**!
     * \brief Refresh the speed configuration to be called whenever setting a
     *        new move to override the default speed
     */
    void refreshSpeeds()
    {
        m_IsMovingLine = m_IsMovingLine && isMoving();
        /*
         * Note: As the pulses per meter are different for each of the three
         * motors the resulting interpolation pulse rate has a vary depending on
         * the angle of the movement for the gantry to move along the line at
         * the specified speed. To do this the set speed has to be converted
         * into the three directional speeds (based on the trajectory of the
         * gantry). These directional speeds can then be used to calculate the
         * pulserate corresponding to the speed at this angle which in turn is
         * then sent to the register containing the set speed of the x-axis as
         * the gantry uses the pulserate in this register as the interpolation
         * pulse rate.
         */
        if (m_IsMovingLine) {
            std::array<double, 2> angles = calcMoveAngles();

            DWORD intSpeed = (DWORD) round(interpolationVelocity.value() * ((sin(angles[1]) / MetersPerSecondPerPulseRate[2]) + cos(angles[1]) * ((cos(angles[0]) / MetersPerSecondPerPulseRate[0]) + (sin(angles[0]) / MetersPerSecondPerPulseRate[1]))));

            LOGD << "Speed " << intSpeed << " " << angles[0] << " " << angles[1];

            checkError(P1240MotChgDV(m_BoardId, X_Axis, intSpeed), "Could not set interpolation speed");
        } else {
            checkError(P1240MotChgDV(m_BoardId, X_Axis, TargetSpeeds[0]), "Could not set x-axis speed");
            checkError(P1240MotChgDV(m_BoardId, Y_Axis, TargetSpeeds[1]), "Could not set y-axis speed");
            checkError(P1240MotChgDV(m_BoardId, Z_Axis, TargetSpeeds[2]), "Could not set z-axis speed");
        }
    }

    void setMove(degree_t xy_angle, degree_t z_angle, millimeter_t lineDistance)
    {
        //angle in normal range (xy: 0 to 360 & z: -90 to 90)
        degree_t xy = xy_angle;
        degree_t z = z_angle;
        if (xy > degree_t(360)) {
            while (xy > degree_t(360)) {
                xy = xy - degree_t(360);
            }
        } else if (xy < degree_t(0)) {
            while (xy < degree_t(0)) {
                xy = xy + degree_t(360);
            }
        }
        if (z > degree_t(90)) {
            while (z > degree_t(90)) {
                z = z - degree_t(90);
            }
        } else if (z < degree_t(-90)) {
            while (z < degree_t(-90)) {
                z = z + degree_t(90);
            }
        }

        std::array<millimeter_t, 3> CurrentPos = getPosition();
        std::array<millimeter_t, 3> targetDist = { 0_mm, 0_mm, 0_mm };

        // Not sure what the significance of this constant is... -- AD
        double z_rad = 0.01745329251994329576923690768489 * z.value();
        double xy_rad = 0.01745329251994329576923690768489 * xy.value();

        targetDist[2] = CurrentPos[2] + millimeter_t(lineDistance.value() * sin(z_rad));
        millimeter_t tDxy = millimeter_t(lineDistance.value() * cos(z_rad));

        targetDist[1] = CurrentPos[1] + millimeter_t(tDxy.value() * sin(xy_rad));
        targetDist[0] = CurrentPos[0] + millimeter_t(tDxy.value() * cos(xy_rad));

        targetDist[0] = millimeter_t(round(targetDist[0].value()));
        targetDist[1] = millimeter_t(round(targetDist[1].value()));
        targetDist[2] = millimeter_t(round(targetDist[2].value()));

        LOGD << "Target position: " << targetDist[0] << " " << targetDist[1]
             << " " << targetDist[2];

        if (targetDist[0] > Limits[0] || targetDist[0] < 0_mm) {
            if (targetDist[0] > Limits[0]) {
                targetDist[0] = Limits[0];
            } else if (targetDist[0] < 0_mm) {
                targetDist[0] = 0_mm;
            }
            targetDist[1] = CurrentPos[1] + millimeter_t((targetDist[0].value() - CurrentPos[0].value()) * tan(xy_rad));
            tDxy = millimeter_t((targetDist[0].value() - CurrentPos[0].value()) / cos(xy_rad));

            targetDist[2] = CurrentPos[2] + millimeter_t(tDxy.value() * tan(z_rad));

            targetDist[0] = millimeter_t(round(targetDist[0].value()));
            targetDist[1] = millimeter_t(round(targetDist[1].value()));
            targetDist[2] = millimeter_t(round(targetDist[2].value()));
        }

        if (targetDist[1] > Limits[1] || targetDist[1] < 0_mm) {
            if (targetDist[1] > Limits[1]) {
                targetDist[1] = Limits[1];
            } else if (targetDist[1] < 0_mm) {
                targetDist[1] = 0_mm;
            }
            targetDist[0] = CurrentPos[0] + millimeter_t((targetDist[1].value() - CurrentPos[1].value()) / tan(xy_rad));
            tDxy = millimeter_t((targetDist[0].value() - CurrentPos[0].value()) / cos(xy_rad));

            targetDist[2] = CurrentPos[2] + millimeter_t(tDxy.value() * tan(z_rad));

            targetDist[0] = millimeter_t(round(targetDist[0].value()));
            targetDist[1] = millimeter_t(round(targetDist[1].value()));
            targetDist[2] = millimeter_t(round(targetDist[2].value()));
        }

        if (targetDist[2] > Limits[2] || targetDist[2] < 0_mm) {
            if (targetDist[2] > Limits[2]) {
                targetDist[2] = Limits[2];
            } else if (targetDist[2] < 0_mm) {
                targetDist[2] = 0_mm;
            }
            tDxy = millimeter_t((targetDist[2].value() - CurrentPos[2].value()) / tan(z_rad));
            targetDist[1] = CurrentPos[1] + millimeter_t(tDxy.value() * sin(xy_rad));
            targetDist[0] = CurrentPos[0] + millimeter_t(tDxy.value() * cos(xy_rad));

            targetDist[0] = millimeter_t(round(targetDist[0].value()));
            targetDist[1] = millimeter_t(round(targetDist[1].value()));
            targetDist[2] = millimeter_t(round(targetDist[2].value()));
        }
        LOGD << "Capped target position: " << targetDist[0] << " "
             << targetDist[1] << " " << targetDist[2];

        P1240MotStop(m_BoardId, XYZ_Axis, XYZ_Axis);

        LOGD << "Final position: " << targetDist[0] << " " << targetDist[1]
             << " " << targetDist[2] << "\n";

        setPosition(targetDist[0], targetDist[1], targetDist[2]);
    }

    void setMove(meters_per_second_t velocity, degree_t xy_angle, degree_t z_angle, millimeter_t lineDistance)
    {
        setSpeed(velocity);
        setMove(xy_angle, z_angle, lineDistance);
    }

    void setMove(degree_t xy_angle, degree_t z_angle)
    {
        setMove(xy_angle, z_angle, Limits[0] + Limits[1] + Limits[2]);
    }

    void setMove(meters_per_second_t velocity, degree_t xy_angle, degree_t z_angle)
    {
        setSpeed(velocity);
        setMove(xy_angle, z_angle);
    }

    void setMove(degree_t xy_angle)
    {
        setMove(xy_angle, degree_t(0));
    }

    void setMove(meters_per_second_t velocity, degree_t xy_angle)
    {
        setSpeed(velocity);
        setMove(xy_angle);
    }

    void setMove(degree_t xy_angle, millimeter_t lineDistance)
    {
        setMove(xy_angle, degree_t(0), lineDistance);
    }

    void setMove(meters_per_second_t velocity, degree_t xy_angle, millimeter_t lineDistance)
    {
        setSpeed(velocity);
        setMove(xy_angle, lineDistance);
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
    void waitToStopMoving(BYTE axis = XYZ_Axis)
    {
        LOGI << "Move start";

        // Repeatedly poll card to check whether gantry is moving
        while (isMoving(axis)) {

            std::array<meters_per_second_t, 3> velo = getVelocity();
            LOGD << "Velocity: " << velo[0] << ", " << velo[1] << ", " << velo[2];

            std::this_thread::sleep_for(10ms);
        }
        LOGI << "Move stop";

        /*
         * If the emergency button is pressed, the gantry will have
         * stopped moving, so we should check the button so we can
         * terminate gracefully.
         */
        checkEmergencyButton();
    }

    /**!
     * \brief Perform a speed test of all 3 axes
     *
     * Moves one meter allowing 50mm for acceleration and deceleration at the
     * beginning and end of the movement. The reference pulse rate can be set;
     * the measures used were taken at a pulserate of 100.
     */
    void speedChecker(int pulses)
    {
        std::array<bool, 3> started = { false, false, false };
        std::array<bool, 3> ended = { false, false, false };
        std::array<double, 3> Pos = { 0, 0, 0 };
        std::array<std::chrono::milliseconds, 3> startms = { std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()) };
        std::array<std::chrono::milliseconds, 3> endms = { std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()) };

        setPosition(1100_mm, 1100_mm, 1100_mm);

        checkError(P1240MotChgDV(m_BoardId, X_Axis, pulses), "Could not set axis speed");
        checkError(P1240MotChgDV(m_BoardId, Y_Axis, pulses), "Could not set axis speed");
        checkError(P1240MotChgDV(m_BoardId, Z_Axis, pulses), "Could not set axis speed");

        while (isMoving(XYZ_Axis)) {
            Vector3<millimeter_t> pos = getPosition();
            if (pos[0].value() > 49.9999999999999 && !started[0]) {
                startms[0] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                Pos[0] = -pos[0].value();
                started[0] = true;
            }

            else if (pos[0].value() > 1049.9999999999 && !ended[0]) {
                endms[0] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                Pos[0] = pos[0].value() + Pos[0];
                ended[0] = true;
                std::cout << "Gantry (x-axis) covered distance of " << Pos[0] << " mm in " << endms[0].operator-=(startms[0]).count() << " milliseconds when set to run at " << pulses << " pulses." << std::endl;
            }

            if (pos[1].value() > 49.9999999999999 && !started[1]) {
                startms[1] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                Pos[1] = -pos[1].value();
                started[1] = true;
            }

            else if (pos[1].value() > 1049.9999999999 && !ended[1]) {
                endms[1] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                Pos[1] = pos[1].value() + Pos[1];
                ended[1] = true;
                std::cout << "Gantry (y-axis) covered distance of " << Pos[1] << " mm in " << endms[1].operator-=(startms[1]).count() << " milliseconds when set to run at " << pulses << " pulses." << std::endl;
            }

            if (pos[2].value() > 49.9999999999999 && !started[2]) {
                startms[2] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                Pos[2] = -pos[2].value();
                started[2] = true;
            }

            else if (pos[2].value() > 1049.9999999999 && !ended[2]) {
                endms[2] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                Pos[2] = pos[2].value() + Pos[2];
                ended[2] = true;
                std::cout << "Gantry (z-axis) covered distance of " << Pos[2] << " mm in " << endms[2].operator-=(startms[2]).count() << " milliseconds when set to run at " << pulses << " pulses." << std::endl;
            }

            std::this_thread::sleep_for(1ms);
        }
    }

private:
    BYTE m_BoardId;
    bool m_IsMovingLine = false;
    std::array<DWORD, 3> TargetSpeeds = { 1200, 1200, 1200 }; //currently set speed in pulse rate
    meters_per_second_t interpolationVelocity = 1_m / 5_s;
    std::array<millimeter_t, 3> LastLineTarget = { 0_mm, 0_mm, 0_mm }; //used to determine the axis speeds when using line movement
    std::array<double, 3> MetersPerSecondPerPulseRate = {
        /*x: */ 0.00007078890696,                      //The x-axis in a test covered 1000.01 mm in 369361 ms at a pulse rate of 100  0.00007078890696
        /*y: */ 0.00006473947430178168828507012561122, //The y-axis in a test covered 1000.10 mm in 369388 ms at a pulse rate of 100
        /*z: */ 0.00004032951665,                      //The z-axis in a test covered 1000.03 mm in 369388 ms at a pulse rate of 100
    };

    /*
     * These values are for converting from a number of motor pulses in the x, y
     * and z axes to a number of millimetres. I took them directly from Chris
     * Johnson's Matlab code and I assume he just measured them empirically.
     * They seem pretty accurate. -- AD
     */
    static constexpr std::array<double, 3> PulsesPerMillimetre = { 7.49625, 8.19672, 13.15789 };

    // These are the gantry's upper x, y and z limits (i.e. the size of the "arena")
    static constexpr std::array<millimeter_t, 3> Limits = { 2996_mm, 1793_mm, 1200_mm }; //problem with z= 1203_mm while it can reach this position it will not move from it (reset and rehome)

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

    template<class UnitType, class InitialUnit = millimeter_t, class T>
    static auto pulsesToUnit5(const std::array<T, 5> &pulses)
    {
        std::array<UnitType, 5> unitArray;
        std::transform(pulses.begin(), pulses.end(), PulsesPerMillimetre.begin(), unitArray.begin(), [](T pulse, double permm) {
            return units::make_unit<InitialUnit>((double) pulse / permm);
        });
        return unitArray;
    }
}; // Gantry
} // Robots
} // BoBRobotics
