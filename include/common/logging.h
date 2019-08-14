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
    Logger(plog::Severity defaultLogLevel);
}; // Logger

#ifndef NO_HEADER_DEFINITIONS
/*
 * We use this dummy object with global scope so that logging is initialised
 * before main() is called.
 */
Logger &loggerObject = Logger::getInstance();
#endif

} // Logging
} // BoBRobotics
