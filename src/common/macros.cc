// BoB robotics includes
#include "common/macros.h"

#ifdef _WIN32
// BoB robotics includes
#include "plog/Log.h"
#endif

namespace BoBRobotics {
AssertionFailedException::AssertionFailedException(const std::string &test, const std::string &file, int line)
  : std::runtime_error("Assertion failed: " + test + " (in " + file + ":" + std::to_string(line) + ":0)")
{
#ifdef _WIN32
    LOG_FATAL << what();
#endif
}

NotImplementedException::NotImplementedException(const std::string &functionName)
  : std::runtime_error(functionName + "() is not implemented for this class")
{
#ifdef _WIN32
    LOG_FATAL << what();
#endif
}
} // BoBRobotics
