#pragma once

// Third-party includes
#include "plog/Appenders/ColorConsoleAppender.h"
#include "plog/Log.h"

// Standard C includes
#include <cstdlib>
#include <cstring>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Logging {
#ifdef DEBUG
constexpr plog::Severity DefaultLogLevel = plog::Severity::debug;
#else
constexpr plog::Severity DefaultLogLevel = plog::Severity::info;
#endif

class Logger
{
public:
    static Logger &getInstance()
    {
        static Logger logger;
        return logger;
    }

private:
    Logger()
    {
        // Get log level
        const auto severity = getLogLevel();

        // Get filename to log to, if specified in env var
        char *filenameEnvVar = std::getenv("LOG_FILE");
        plog::util::nstring filename;
        if (filenameEnvVar) {
            // String conversion is necessary for Windows (which uses wchar_t)
            filename = filenameEnvVar;
        }

        // Do initialisation
        plog::init(severity, filename.c_str());

        /*
         * Also print log messages to console in colour.
         *
         * The plog documentation reports that this should have static scope.
         */
        static plog::ColorConsoleAppender<plog::TxtFormatter> appender;
        plog::get()->addAppender(&appender);
    }

    static constexpr auto logLevelToString(plog::Severity severity)
    {
        switch (severity) {
        case plog::fatal:
            return "FATAL";
        case plog::error:
            return "ERROR";
        case plog::warning:
            return "WARNING";
        case plog::info:
            return "INFO";
        case plog::debug:
            return "DEBUG";
        case plog::verbose:
            return "VERBOSE";
        default:
            return "NONE";
        }
    }

    // Get log level from env var, or set default
    static plog::Severity getLogLevel()
    {
        // Try to get from environment variable
        char *severityEnvVar = std::getenv("LOG_LEVEL");
        if (!severityEnvVar) {
            return DefaultLogLevel;
        }

        // Convert to uppsercase
        for (char *c = severityEnvVar; *c; c++) {
            *c = std::toupper(*c);
        }

        // Match to enum member
        for (plog::Severity severity = plog::none; severity <= plog::verbose; severity = static_cast<plog::Severity>(severity + 1)) {
            if (std::strcmp(logLevelToString(severity),
                            severityEnvVar) == 0) {
                return severity;
            }
        }

        throw std::runtime_error(std::string(severityEnvVar) + " is not a valid log level");
    }
}; // Logger

/*
 * We use this dummy object with global scope so that logging is initialised
 * before main() is called.
 */
#ifndef NO_HEADER_DEFINITIONS
Logger &loggerObject = Logger::getInstance();
#endif
} // Logging
} // BoBRobotics
