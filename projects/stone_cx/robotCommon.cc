#define NO_HEADER_DEFINITIONS

#include "robotCommon.h"

// BoB robotics includes
#include "common/logging.h"
#include "robots/robot_type.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Standard C++ includes
#include <algorithm>
#include <numeric>

//---------------------------------------------------------------------------
// StoneCX::IMULM9DS1
//---------------------------------------------------------------------------
BoBRobotics::IMULM9DS1::IMULM9DS1()
{
    // Initialise IMU magnetometer was default settings
    LM9DS1::MagnetoSettings magSettings;
    m_IMU.initMagneto(magSettings);
}

float BoBRobotics::IMULM9DS1::getHeading()
{
    // Wait for magneto to become available
    while(!m_IMU.isMagnetoAvailable()){
    }

    // Read magneto
    float magnetoData[3];
    m_IMU.readMagneto(magnetoData);

    // Calculate heading angle from magneto data and set atomic value
    return atan2(magnetoData[0], magnetoData[2]);
}

//---------------------------------------------------------------------------
// StoneCX::IMUBN055
//---------------------------------------------------------------------------
float BoBRobotics::IMUBN055::getHeading()
{
    using namespace units::angle;
    
    const degree_t headingDegrees{m_IMU.getVector()[0]};
    return static_cast<radian_t>(headingDegrees);
}

//---------------------------------------------------------------------------
// StoneCX::IMUEV3
//---------------------------------------------------------------------------
BoBRobotics::IMUEV3::IMUEV3(Net::Connection &connection)
:   m_NetSource(connection)
{
}

float BoBRobotics::IMUEV3::getHeading()
{
    using namespace units::angle;
    return static_cast<radian_t>(m_NetSource.getYaw()).value();
}



float BoBRobotics::StoneCX::driveMotorFromCPU1(BoBRobotics::Robots::Tank &motor, bool display)
{
    
}