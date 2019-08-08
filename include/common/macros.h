#pragma once

// Standard C++ includes
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
    AssertionFailedException(const std::string &test, const std::string &file, int line);
}; // AssertionFailedException

class NotImplementedException
  : public std::runtime_error
{
public:
    NotImplementedException(const std::string &functionName);
}; // NotImplementedException
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

/**!
 * \brief Defines a packed struct in MSVC or GNU-type compilers.
 *
 * NB: Don't define your struct with commas in or it'll break the macro.
 */
#ifdef __GNUC__
#define BOB_PACKED(class_to_pack) class_to_pack __attribute__((__packed__))
#else
#define BOB_PACKED(class_to_pack) __pragma(pack(push, 1)) class_to_pack __pragma(pack(pop))
#endif

#define BOB_NOT_IMPLEMENTED(funcdef)                          \
    funcdef                                                   \
    {                                                         \
        throw BoBRobotics::NotImplementedException(__func__); \
    }
