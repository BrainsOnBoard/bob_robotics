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
#endif
