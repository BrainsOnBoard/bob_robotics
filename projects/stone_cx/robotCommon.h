#pragma once

// BoB robotics includes
#include "common/bn055_imu.h"
#include "common/lm9ds1_imu.h"
#include "net/imu_netsource.h"


namespace BoBRobotics {
// Forward declarations
namespace Robots 
{
class Tank;
class ROBOT_TYPE;
}

//---------------------------------------------------------------------------
// StoneCX::IMU
//---------------------------------------------------------------------------
namespace StoneCX {    




// Functions
float driveMotorFromCPU1(BoBRobotics::Robots::Tank &motor, bool display = false);
}
}