// BoB robotics includes
#include "common/main.h"

// Third-party includes
#include "plog/Log.h"
#include "plog/Appenders/ColorConsoleAppender.h"

// Standard C++ includes
#include <exception>

// Standard C includes
#include <cstring>

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
#ifdef DEBUG
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

    auto commitSev = plog::Severity::debug;
#ifdef BOB_IS_EXPERIMENT
    commitSev = plog::Severity::info; // Always print this info for an experiment
    bool good = true;

    LOGI << "Code compiled with IS_EXPERIMENT=TRUE; doing extra checks to see "
            "if results will be suitable for publication...";

#ifdef BOB_GIT_FAILED
    good = false;

    LOGE << "CMake was unable to run git, so there will be no record of which "
            "version of the code is in use! Please download BoB robotics using "
            "git clone.";
#else
#ifdef BOB_GIT_TREE_DIRTY
    good = false;
    LOGE << "The git working tree is dirty! You should stash or commit your "
            "changes then rebuild this program for a real experiment. (The "
            "reason is that otherwise there will be no record of exactly which "
            "version of the code was used and so it cannot be guaranteed that "
            "the experiment(s) can be repeated.)";
#endif // BOB_GIT_TREE_DIRTY

#ifdef BOB_GIT_COMMIT_NOT_IN_MASTER
    good = false;
    LOGE << "The current git commit for this code is not in the master branch. "
            "All code used in publications must be reviewed and merged before "
            "being used to collect data!";
#endif // BOB_GIT_COMMIT_NOT_IN_MASTER

#endif // BOB_GIT_FAILED

#ifdef DEBUG
    good = false;
    LOGW << "This code is a debug build. You almost certainly want to rebuild "
            "with cmake -DCMAKE_BUILD_TYPE=Release for experiments.";
#endif // DEBUG

    // Give some feedback in the case that no errors were printed
    if (good) {
        LOGI << "Extra experiment checks passed 🥳!";
    }

#endif // BOB_IS_EXPERIMENT

    LOG(commitSev) << "BoB robotics git commit: " BOB_ROBOTICS_GIT_COMMIT;

    /*
     * BoB robotics could be being used as an external library, in which case
     * there may be a different git tree in which the current program lives.
     */
    if (strcmp(BOB_ROBOTICS_GIT_COMMIT, BOB_PROJECT_GIT_COMMIT) != 0) {
        LOG(commitSev) << "Project git commit: " BOB_PROJECT_GIT_COMMIT;
    }

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
