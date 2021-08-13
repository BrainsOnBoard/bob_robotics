#pragma once

// Include appropriate header, depending on what kind of robot the user wants
#if defined(ROBOT_TYPE_NORBOT)
#include "norbot.h"
#elif defined(ROBOT_TYPE_EV3)
#include "ev3/ev3.h"
#elif defined(ROBOT_TYPE_SURVEYOR)
#include "surveyor.h"
#elif defined(ROBOT_TYPE_BUNDLEDTANKNETSINK)
#include "tank_netsink.h"
#elif defined(ROBOT_TYPE_TANK)
#include "tank.h" // For dummy tank
#elif defined(ROBOT_TYPE_MECANUM)
#include "mecanum.h"
#elif defined(ROBOT_TYPE_MECANUMPCA9685)
#include "mecanum_pca_9685.h"
#elif defined(ROBOT_TYPE_GAZEBO_TANK)
#include "gazebo/tank.h"
#endif
