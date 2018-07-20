#pragma once
/*
 * This header is needed, because I'm getting compile errors when trying to include
 * OpenCV because the KEY_UP and KEY_DOWN macros are already defined (on Ubuntu 18.04).
 * 	- AD
 */
#ifdef KEY_UP
#undef KEY_UP
#endif
#ifdef KEY_DOWN
#undef KEY_DOWN
#endif
#include "../common/opencv.h"
