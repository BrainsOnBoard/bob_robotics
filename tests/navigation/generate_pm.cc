#include "generate_data.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"

int
bobMain(int, char **)
{
    generateData<BoBRobotics::Navigation::PerfectMemoryRotater<>>("pm.bin");
    return EXIT_SUCCESS;
}
