#pragma once

// For IDEs
#ifndef ROBOT_TYPE
#define ROBOT_TYPE_TANK_DUMMYTANK
#define ROBOT_TYPE_NAMESPACE BoBRobotics::Robots::Tank
#define ROBOT_TYPE ROBOT_TYPE_NAMESPACE::DummyTank
#endif

// Include appropriate header, depending on what kind of robot the user wants
#if defined(ROBOT_TYPE_TANK_NORBOT)
#include "tank/norbot.h"
#elif defined(ROBOT_TYPE_EV3_EV3)
#include "ev3/ev3.h"
#elif defined(ROBOT_TYPE_TANK_BUNDLEDTANKNETSINK)
#include "tank/net/sink.h"
#elif defined(ROBOT_TYPE_TANK_DUMMYTANK)
#include "tank/dummy_tank.h" // For dummy tank
#elif defined(ROBOT_TYPE_OMNI2D_MECANUM)
#include "omni2d/mecanum.h"
#elif defined(ROBOT_TYPE_OMNI2D_MECANUMPCA9685)
#include "mecanum_pca_9685.h"
#elif defined(ROBOT_TYPE_GAZEBO_TANK)
#include "gazebo/tank.h"
#endif
