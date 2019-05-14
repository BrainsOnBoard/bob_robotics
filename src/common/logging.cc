// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"

// Third-party includes
#include "plog/Appenders/ColorConsoleAppender.h"

// Standard C includes
#include <cstdlib>
#include <cstring>

// Standard C++ includes
#include <string>

namespace {
// Plog's default TxtFormatter is a bit verbose, so let's implement our own
struct Formatter
{
    static plog::util::nstring header()
    {
        return plog::util::nstring();
    }

    static plog::util::nstring format(const plog::Record &record)
    {
        tm t;
        plog::util::localtime_s(&t, &record.getTime().time);
        plog::util::nostringstream ss;
        ss << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_hour << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_min << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_sec << PLOG_NSTR(".") << std::setfill(PLOG_NSTR('0')) << std::setw(3) << record.getTime().millitm << PLOG_NSTR(" ");
        ss << PLOG_NSTR("[") << record.getFunc() << PLOG_NSTR("@") << record.getLine() << PLOG_NSTR("] ");
        const auto severity = record.getSeverity();
        if (severity != plog::Severity::info && severity != plog::Severity::verbose && severity != plog::Severity::none) {
            ss << plog::severityToString(severity) << PLOG_NSTR(": ");
        }
        ss << record.getMessage() << PLOG_NSTR("\n");

        return ss.str();
    }
};

static constexpr auto
logLevelToString(plog::Severity severity)
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
static plog::Severity
getLogLevel(plog::Severity defaultLogLevel)
{
    // Try to get from environment variable
    char *severityEnvVar = std::getenv("LOG_LEVEL");
    if (!severityEnvVar) {
        return defaultLogLevel;
    }

    // Convert to uppercase
    for (char *c = severityEnvVar; *c != '\0'; c++) {
        *c = toupper(*c);
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
}

namespace BoBRobotics {
namespace Logging {
Logger::Logger(plog::Severity defaultLogLevel)
{
    // Get log level
    const auto severity = getLogLevel(defaultLogLevel);

    // Get filename to log to, if specified in env var
    char *filenameEnvVar = std::getenv("LOG_FILE");
    plog::util::nstring filename;
    if (filenameEnvVar) {
        // String conversion is necessary for Windows (which uses wchar_t)
        filename = plog::util::nstring{ &filenameEnvVar[0], &filenameEnvVar[strlen(filenameEnvVar)] };
    }

    // Do initialisation
    plog::init(severity, filename.c_str());

    /*
         * Also print log messages to console in colour.
         *
         * The plog documentation reports that this should have static scope.
         */
    static plog::ColorConsoleAppender<Formatter> appender;
    plog::get()->addAppender(&appender);
}

Logger &
Logger::getInstance()
{
    static Logger logger(DefaultLogLevel);
    return logger;
}

} // Logging
} // BoBRobotics
