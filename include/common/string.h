#pragma once

// Standard C++ includes
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace BoBRobotics {
void
strTrimLeft(std::string &);

void
strTrimRight(std::string &);

void
strTrim(std::string &);

std::vector<std::string>
strSplit(const std::string &s, char delim);

void
strSplit(const std::string &s, char delim, std::vector<std::string> &parts);

//! writes a floating point value to a string - setting the precision so no digits are lost
template<class T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
std::string writePreciseString(T value)
{
    std::ostringstream s;

    // Set scientific formatting
    s << std::scientific;

    // Set precision to what is required to fully represent T
    s << std::setprecision(std::numeric_limits<T>::max_digits10);

    // Write value to stream
    s << value;

    return s.str();
}
} // BoBRobotics
