#pragma once

// Standard C++ includes
#include <iostream>
#include <stdexcept>
#include <string>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::AssertionFailedException
//----------------------------------------------------------------------------
//! Exception class used by BOB_ASSERT macro
class AssertionFailedException
  : public std::runtime_error
{
public:
    AssertionFailedException(const std::string &test, const std::string &file, int line)
      : std::runtime_error("Assertion failed: " + test + " (in " + file + " at line " + std::to_string(line) + ")")
    {
#ifdef _WIN32
        std::cout << what();
#endif
    }
}; // AssertionFailedException
} // BoBRobotics

/**!
 * \brief If EXPRESSION evaluates to false, throw AssertionFailedException
 *
 * The advantage of this macro over assert in <cassert> is that it throws an
 * exception rather than just terminating the program. This is especially useful
 * when controlling robots as we want exceptions to be caught so that the robot
 * objects' destructors are called and they can be stopped before they bang into
 * a wall.
 */
#define BOB_ASSERT(EXPRESSION)                                                        \
    if (!(EXPRESSION)) {                                                              \
        throw BoBRobotics::AssertionFailedException(#EXPRESSION, __FILE__, __LINE__); \
    }
