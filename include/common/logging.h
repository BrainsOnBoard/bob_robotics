#pragma once

// Third-party includes
#define assert BOB_ASSERT
#include "plog/Log.h"
#undef assert

namespace BoBRobotics {
namespace Logging {
#ifdef DEBUG
constexpr plog::Severity DefaultLogLevel = plog::Severity::debug;
#else
constexpr plog::Severity DefaultLogLevel = plog::Severity::info;
#endif

//! Dummy class for initialising logging before main() is called
class Logger
{
public:
    static Logger &getInstance()
    {
        static Logger logger(DefaultLogLevel);
        return logger;
    }

private:
    Logger(plog::Severity defaultLogLevel);
}; // Logger
} // Logging
} // BoBRobotics
