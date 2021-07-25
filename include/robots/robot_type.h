#pragma once

// Include appropriate header, depending on what kind of robot the user wants
#if defined(ROBOT_TYPE_NORBOT)
#include "tank/norbot.h"
#elif defined(ROBOT_TYPE_EV3)
#include "tank/ev3/ev3.h"
#elif defined(ROBOT_TYPE_SURVEYOR)
#include "tank/surveyor.h"
#elif defined(ROBOT_TYPE_BUNDLEDTANKNETSINK)
#include "tank/tank_netsink.h"
#elif defined(ROBOT_TYPE_TANK)
#include "tank/tank.h" // For dummy tank
#elif defined(ROBOT_TYPE_MECANUM)
#include "mecanum.h"
#elif defined(ROBOT_TYPE_GAZEBO_TANK)
#include "gazebo/tank.h"
#endif
