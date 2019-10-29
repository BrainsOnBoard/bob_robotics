#pragma once

// Include appropriate header, depending on what kind of robot the user wants
#if defined(ROBOT_TYPE_Norbot)
#include "norbot.h"
#elif defined(ROBOT_TYPE_EV3)
#include "ev3/ev3.h"
#elif defined(ROBOT_TYPE_Surveyor)
#include "surveyor.h"
#elif defined(ROBOT_TYPE_BundledTankNetSink)
#include "tank_netsink.h"
#endif
