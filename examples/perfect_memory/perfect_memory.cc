// BoB robotics includes
#include "common/path.h"
#include "common/timer.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_raw.h"
#include "navigation/perfect_memory_store_hog.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <algorithm>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

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
        pm.trainRoute(snapshots);

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
        pm.trainRoute(snapshots);

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
        pm.trainRoute(snapshots);

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
        pm.trainRoute(snapshots);

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
        pm.trainRoute(snapshots);

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
        pm.trainRoute(snapshots);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        cv::Mat snap = cv::imread((Path::getRepoPath() / "tools/ant_world_db_creator/ant1_route1/image00010.png").str(), cv::IMREAD_GRAYSCALE);
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
