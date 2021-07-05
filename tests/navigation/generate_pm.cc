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

    // The HOG code doesn't support using a mask, so don't try
    generateDataWindow<PerfectMemoryRotater<PerfectMemoryStore::HOG<>>>("pm_hog.bin", {}, cv::Size(10, 10), 8);
    generateDataWindow<PerfectMemoryRotater<PerfectMemoryStore::HOG<CorrCoefficient>>>("pm_hog_ccoeff.bin", {}, cv::Size(10, 10), 8);

    generateDataWindow<PerfectMemoryRotater<PerfectMemoryStore::RawImage<CorrCoefficient>>>("pm_ccoeff.bin", {});
#ifdef BOB_OPENCV_SUPPORTS_CCOEFF_MASKS
    generateDataWindow<PerfectMemoryRotater<PerfectMemoryStore::RawImage<CorrCoefficient>>>("mask_pm_ccoeff.bin", TestMask);
#else
    LOGW << "Not generating mask_pm_ccoeff.bin as versions of OpenCV older than 4.5.2 have a bug (your version: " CV_VERSION ")";
#endif
    return EXIT_SUCCESS;
}
