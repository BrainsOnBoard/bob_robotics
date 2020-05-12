#pragma once

// Third-party includes
#include "plog/Log.h"

namespace BoBRobotics {
namespace Logging {
#if defined(DEBUG) || defined(_DEBUG)
constexpr plog::Severity DefaultLogLevel = plog::Severity::debug;
#else
constexpr plog::Severity DefaultLogLevel = plog::Severity::info;
#endif

//! Dummy class for initialising logging before main() is called
class Logger
{
public:
    static Logger &getInstance();

private:
    Logger(plog::Severity defaultLogLevel = DefaultLogLevel);
    static Logger logger;
}; // Logger


} // Logging
} // BoBRobotics
