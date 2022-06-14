// BoB robotics includes
#include "common/string.h"

// Standard C++ includes
#include <algorithm>
#include <sstream>

// Standard C includes
#include <cctype>

namespace BoBRobotics {
template<class Iter>
auto findNonSpace(Iter begin, Iter end)
{
    const auto notSpace = [](char c) {
        return !std::isspace(c);
    };
    return std::find_if(begin, end, notSpace);
}

void strTrimLeft(std::string &s)
{
    s.erase(s.begin(), findNonSpace(s.begin(), s.end()));
}

void strTrimRight(std::string &s)
{
    const auto nonSpace = findNonSpace(s.rbegin(), s.rend());
    s.erase(nonSpace.base(), s.rbegin().base());
}

void strTrim(std::string &s)
{
    strTrimLeft(s);
    strTrimRight(s);
}

std::vector<std::string>
strSplit(const std::string &s, char delim)
{
    std::vector<std::string> parts;
    strSplit(s, delim, parts);
    return parts;
}

void
strSplit(const std::string &s, char delim, std::vector<std::string> &parts)
{
    parts.clear();

    // Treat empty strings as containing no fields
    if (s.empty()) {
        return;
    }

    auto start = s.c_str();
    auto c = start;
    for (; *c != '\0'; c++) {
        if (*c == delim) {
            parts.emplace_back(start, c);
            start = c + 1;
        }
    }

    // Final field is terminated with EOL
    parts.emplace_back(start, c);
}
} // BoBRobotics
