// Standard C++ includes
#include <iostream>

// BoB robotics includes
#include "common/timer.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_hog.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

template<typename T>
void
loadSnapshots(T &pm)
{
    // Load snapshots
    pm.loadSnapshotsFromPath("../../tools/ant_world_db_creator/ant1_route1", true);
    std::cout << "Loaded " << pm.getNumSnapshots() << " snapshots" << std::endl;
}

int
main()
{
    const cv::Size imSize(180, 50);
    {
        std::cout << "Testing with best-matching snapshot method..." << std::endl;

        // Default algorithm: find best-matching snapshot, use abs diff
        PerfectMemory<> pm(imSize);
        loadSnapshots(pm);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        degree_t heading;
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;
    }

    {
        std::cout << std::endl << "Testing with RMS image difference..." << std::endl;
        PerfectMemory<BestMatchingSnapshot, RMSDiff> pm(imSize);
        loadSnapshots(pm);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        degree_t heading;
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;
    }

    {
        constexpr size_t numSnapshots = 3;
        std::cout << std::endl <<  "Testing with " << numSnapshots << " weighted snapshots..." << std::endl;
        PerfectMemory<WeightNSnapshots<numSnapshots>> pm(imSize);
        loadSnapshots(pm);

        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        const auto snap = pm.getSnapshot(10);
        degree_t heading;
        std::array<size_t, numSnapshots> snapshots;
        std::array<float, numSnapshots> differences;
        std::tie(heading, snapshots, differences) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        for (size_t i = 0; i < snapshots.size(); i++) {
            std::cout << "Snapshot " << i + 1 << ": #" << snapshots[i]
                      << " (" << differences[i] << ")" << std::endl;
        }
    }

    {
        std::cout << std::endl << "Testing with HOG..." << std::endl;

        PerfectMemoryHOG<> pm(imSize);
        loadSnapshots(pm);

        // Time testing phase
        Timer<> t{ "Time taken for testing: " };

        // Treat snapshot #10 as test data
        cv::Mat snap = cv::imread("../../tools/ant_world_db_creator/ant1_route1/image_00010.png", CV_LOAD_IMAGE_GRAYSCALE);
        cv::resize(snap, snap, imSize);
        degree_t heading;
        size_t snapshot;
        float difference;
        std::tie(heading, snapshot, difference) = pm.getHeading(snap);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;
    }
}