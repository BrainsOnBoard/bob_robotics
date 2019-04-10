// BoB robotics includes
#include "common/logging.h"
#include "common/timer.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_raw.h"
#include "navigation/perfect_memory_store_hog.h"

// Standard C++ includes
#include <algorithm>
#include <iostream>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

template<typename T>
void
trainRoute(T &pm)
{
    // Load snapshots
    pm.trainRoute("../../tools/ant_world_db_creator/ant1_route1", true);
    std::cout << "Loaded " << pm.getNumSnapshots() << " snapshots" << std::endl;
}

int
main()
{
    const cv::Size imSize(180, 50);
    units::angle::degree_t heading;
    std::vector<std::vector<float>> allDifferences;

    {
        std::cout << "Testing with best-matching snapshot method..." << std::endl;

        // Default algorithm: find best-matching snapshot, use abs diff
        PerfectMemoryRotater<> pm(imSize);
        trainRoute(pm);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;
    }

    {
        std::cout << "Testing with best-matching snapshot method with partial rotation..." << std::endl;

        // Default algorithm: find best-matching snapshot, use abs diff
        PerfectMemoryRotater<> pm(imSize);
        trainRoute(pm);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        std::vector<size_t> rotations(snap.cols / 2);
        std::iota(rotations.begin(), rotations.end(), snap.cols / 2);

        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap, rotations.begin(), rotations.end());
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;
    }

    {
        std::cout << std::endl << "Testing with RMS image difference..." << std::endl;
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<RMSDiff>> pm(imSize);
        trainRoute(pm);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;
    }

    {
        constexpr size_t numComp = 3;
        std::cout << std::endl <<  "Testing with " << numComp << " weighted snapshots..." << std::endl;
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, WeightSnapshotsDynamic<numComp>> pm(imSize);
        trainRoute(pm);

        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        std::array<size_t, numComp> snapshots;
        std::array<float, numComp> differences;
        std::tie(heading, snapshots, differences, allDifferences) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        for (size_t i = 0; i < snapshots.size(); i++) {
            std::cout << "Snapshot " << i + 1 << ": #" << snapshots[i]
                      << " (" << differences[i] << ")" << std::endl;
        }
    }

    {
        std::cout << std::endl << "Testing with HOG..." << std::endl;

        PerfectMemoryRotater<PerfectMemoryStore::HOG<>> pm(imSize, cv::Size(10, 10), 8);
        trainRoute(pm);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        cv::Mat snap = cv::imread("../../tools/ant_world_db_creator/ant1_route1/image_00010.png", cv::IMREAD_GRAYSCALE);
        cv::resize(snap, snap, imSize);
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;
    }

    return EXIT_SUCCESS;
}
