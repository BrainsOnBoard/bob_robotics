#pragma once

// Standard C++ includes
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
} // BoBRobotics
