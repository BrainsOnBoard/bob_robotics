#include "generate_data.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_hog.h"

using namespace BoBRobotics::Navigation;

int
bobMain(int, char **)
{
    generateData<PerfectMemoryRotater<>>("pm.bin");
    generateData<PerfectMemoryRotater<PerfectMemoryStore::RawImage<RMSDiff>>>("pm_rms.bin");
    generateData<PerfectMemoryRotater<PerfectMemoryStore::HOG<>>>("pm_hog.bin", cv::Size(10, 10), 8);
    return EXIT_SUCCESS;
}
