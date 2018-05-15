#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace OS::FileSystem {
bool
fileExists(const std::string &filePath)
{
    struct stat buffer;
    return (stat(filePath.c_str(), &buffer) == 0);
}
}
