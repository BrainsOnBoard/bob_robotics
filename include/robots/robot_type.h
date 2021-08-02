#pragma once

// For IDEs
#ifndef ROBOT_TYPE
#define ROBOT_TYPE_TANK_TANKBASE
#define ROBOT_TYPE Tank::TankBase
#endif

// Include appropriate header, depending on what kind of robot the user wants
#if defined(ROBOT_TYPE_TANK_NORBOT)
#include "tank/norbot.h"
#elif defined(ROBOT_TYPE_EV3_EV3)
#include "tank/ev3/ev3.h"
#elif defined(ROBOT_TYPE_TANK_BUNDLEDTANKNETSINK)
#include "tank/net/sink.h"
#elif defined(ROBOT_TYPE_TANK_TANKBASE)
#include "tank/tank_base.h" // For dummy tank
#elif defined(ROBOT_TYPE_MECANUM)
#include "omni2d/mecanum.h"
#elif defined(ROBOT_TYPE_GAZEBO_TANK)
#include "gazebo/tank.h"
#endif
