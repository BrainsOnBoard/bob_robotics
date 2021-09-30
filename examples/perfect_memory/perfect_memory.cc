// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"
#include "common/timer.h"
#include "navigation/image_database.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_raw.h"
#include "navigation/perfect_memory_store_hog.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <algorithm>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

//------------------------------------------------------------------------
// Anonymous namespace
//------------------------------------------------------------------------
namespace
{
template<typename PM>
void trainRoute(PM &pm, const std::vector<cv::Mat> &images)
{
    for (auto &image : images) {
        pm.train(image);
    }
}
}

int bobMain(int, char **)
{
    const cv::Size imSize(180, 50);
    units::angle::degree_t heading;
    const Eigen::MatrixXf *allDifferences;

    const ImageDatabase imdb{ Path::getRepoPath() / "tools/ant_world_db_creator/ant1_route1" };
    const auto snapshots = imdb.loadImages(imSize);
    LOGI << "Loaded " << snapshots.size() << " snapshots";

    {
        LOGI << "Testing with best-matching snapshot method...";

        // Default algorithm: find best-matching snapshot, use abs diff
        PerfectMemoryRotater<> pm(imSize);
        trainRoute(pm, snapshots);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap);
        LOGI << "Heading: " << heading;
        LOGI << "Best-matching snapshot: #" << snapshot;
        LOGI << "Difference score: " << difference;
    }

    {
        LOGI << "Testing with best-matching snapshot method with partial rotation...";

        // Default algorithm: find best-matching snapshot, use abs diff
        PerfectMemoryRotater<> pm(imSize);
        trainRoute(pm, snapshots);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        std::vector<size_t> rotations(snap.cols / 2);
        std::iota(rotations.begin(), rotations.end(), snap.cols / 2);

        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap, rotations.begin(), rotations.end());
        LOGI << "Heading: " << heading;
        LOGI << "Best-matching snapshot: #" << snapshot;
        LOGI << "Difference score: " << difference;
    }

    {
        LOGI << "Testing with RMS image difference...";
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<RMSDiff>> pm(imSize);
        trainRoute(pm, snapshots);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap);
        LOGI << "Heading: " << heading;
        LOGI << "Best-matching snapshot: #" << snapshot;
        LOGI << "Difference score: " << difference;
    }

    {
        LOGI << "Testing with correlation coefficient difference...";
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<CorrCoefficient>> pm(imSize);
        trainRoute(pm, snapshots);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap);
        LOGI << "Heading: " << heading;
        LOGI << "Best-matching snapshot: #" << snapshot;
        LOGI << "Difference score: " << difference;
    }

    {
        constexpr size_t numComp = 3;
        LOGI <<  "Testing with " << numComp << " weighted snapshots...";
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, WeightSnapshotsDynamic<numComp>> pm(imSize);
        trainRoute(pm, snapshots);

        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        std::array<size_t, numComp> snapshots;
        std::array<float, numComp> differences;
        std::tie(heading, snapshots, differences, allDifferences) = pm.getHeading(snap);
        LOGI << "Heading: " << heading;
        for (size_t i = 0; i < snapshots.size(); i++) {
            LOGI << "Snapshot " << i + 1 << ": #" << snapshots[i]
                      << " (" << differences[i] << ")";
        }
    }

    {
        LOGI << "Testing with HOG...";

        PerfectMemoryRotater<PerfectMemoryStore::HOG<>> pm(imSize, cv::Size(10, 10), 8);
        trainRoute(pm, snapshots);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        cv::Mat snap = cv::imread((Path::getRepoPath() / "tools/ant_world_db_creator/ant1_route1/image_00010.png").str(), cv::IMREAD_GRAYSCALE);
        BOB_ASSERT(!snap.empty());
        cv::resize(snap, snap, imSize);
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap);
        LOGI << "Heading: " << heading;
        LOGI << "Best-matching snapshot: #" << snapshot;
        LOGI << "Difference score: " << difference;
    }

    return EXIT_SUCCESS;
}
