#pragma once

// Third-party includes
#include "plog/Log.h"
#include "plog/Appenders/ColorConsoleAppender.h"

// Standard C includes
#include <cstdlib>

namespace BoBRobotics {
//! Initialises our logging library with default options
inline void initialiseLogging()
{
    // Get log level from env var, or set default
    plog::Severity severity;
    char *severityEnvVar = std::getenv("LOG_LEVEL");
    if (severityEnvVar) {
        for (char *c = severityEnvVar; c; c++) {
            *c = std::toupper(*c);
        }
        severity = plog::severityFromString(severityEnvVar);
    } else {
        severity = plog::Severity::info; // Default log level
    }

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
} // BoBRobotics
