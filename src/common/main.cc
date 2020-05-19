// BoB robotics includes
#include "common/main.h"

// Third-party includes
#include "plog/Log.h"
#include "plog/Appenders/ColorConsoleAppender.h"

// Standard C++ includes
#include <exception>

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

namespace BoBRobotics {
void
initLogging()
{
// Get log level
#if defined(DEBUG) || defined(_DEBUG)
    constexpr plog::Severity defaultLogLevel = plog::Severity::debug;
#else
    constexpr plog::Severity defaultLogLevel = plog::Severity::info;
#endif
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
} // BoBRobotics

#ifndef BOB_SHARED_LIB
// Forward declaration; put definition in your own main C++ file
int bobMain(int argc, char **argv);

int
main(int argc, char **argv)
{
    // We always want plog working
    BoBRobotics::initLogging();

    /*
     * When debugging, it's handy to know exactly which version of the source
     * code we compiled.
     */
    LOGD << "Project git commit: " BOB_PROJECT_GIT_COMMIT;
    LOGD << "BoB robotics git commit: " BOB_ROBOTICS_GIT_COMMIT;

#ifndef DEBUG
    try {
#endif // !DEBUG
        return bobMain(argc, argv);
#ifndef DEBUG
    } catch (std::exception &e) {
#ifdef _WIN32
        // Windows doesn't print exception details by default
        LOG_FATAL << "Uncaught exception: " << e.what();
#endif

        // Rethrow exception so it can be handled by OS's default handler
        throw;
    } catch (...) {
        // Rethrow exceptions not of type std::exception
        throw;
    }
#endif // !DEBUG
}
#endif // !BOB_SHARED_LIB
