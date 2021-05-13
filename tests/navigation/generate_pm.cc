#include "generate_data.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"

using namespace BoBRobotics::Navigation;

int
bobMain(int, char **)
{
    generateData<PerfectMemoryRotater<>>("pm.bin");
    generateData<PerfectMemoryRotater<PerfectMemoryStore::RawImage<RMSDiff>>>("pm_rms.bin");
    return EXIT_SUCCESS;
}
