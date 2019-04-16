// BoB robotics includes
#include "common/logging.h"
#include "navigation/antworld_rotater.h"
#include "navigation/perfect_memory.h"
#include "navigation/plot.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
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
    /*
     * I've set the width of the image to be the same as the (raw) unwrapped
     * images we get from the robot gantry, but the height is greater (cf. 58)
     * because I wanted to keep the aspect ratio as it was (200x40).
     *      -- AD
     */
    const cv::Size RenderSize{ 180, 50 };

    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    renderer.getWorld().load("../../include/antworld/world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Create agent object
    AntWorld::AntAgent agent(window.get(), renderer, RenderSize);
    agent.setPosition(5.5_m, 4_m, 10_mm);

    units::angle::degree_t heading;

    {
        std::cout << "Using ant world rotater..." << std::endl;
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, BestMatchingSnapshot, AntWorldRotater> pm(RenderSize);
        trainRoute(pm);

        size_t snapshot;
        float difference;
        std::vector<std::vector<float>> allDifferences;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(agent, 2_deg);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;

        // Plot RIDF
        plotRIDF(allDifferences[snapshot]);
        std::cout << std::endl;
    }

    {
        std::cout << "Using in silico rotater..." << std::endl;
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, BestMatchingSnapshot, InSilicoRotater> pm(RenderSize);
        trainRoute(pm);

        agent.setAttitude(0_deg, 0_deg, 0_deg);
        cv::Mat fr;
        agent.readGreyscaleFrame(fr);

        size_t snapshot;
        float difference;
        std::vector<std::vector<float>> allDifferences;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(fr);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;

        // Plot RIDF
        plotRIDF(allDifferences[snapshot]);
        std::cout << std::endl;
    }
}
