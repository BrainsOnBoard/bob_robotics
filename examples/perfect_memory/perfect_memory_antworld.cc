// BoB robotics includes
#include "common/path.h"
#include "common/logging.h"
#include "navigation/antworld_rotater.h"
#include "navigation/perfect_memory.h"
#include "navigation/plot.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

template<typename T>
void
trainRoute(T &pm, const filesystem::path &routePath)
{
    // Load snapshots
    pm.trainRoute(routePath, true);
    LOGI << "Loaded " << pm.getNumSnapshots() << " snapshots";
}

int
main(int, char **)
{
    /*
     * I've set the width of the image to be the same as the (raw) unwrapped
     * images we get from the robot gantry, but the height is greater (cf. 58)
     * because I wanted to keep the aspect ratio as it was (200x40).
     *      -- AD
     */
    const cv::Size RenderSize{ 180, 50 };
    const auto routePath = Path::getRepoPath() / "tools" / "ant_world_db_creator" / "ant1_route1";

    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    renderer.getWorld().load(Path::getResourcesPath() / "antworld" / "world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Create agent object
    AntWorld::AntAgent agent(*window, renderer, RenderSize);
    agent.setPosition(5.5_m, 4_m, 10_mm);

    units::angle::degree_t heading;

    {
        LOGI << "Using ant world rotater...";
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, BestMatchingSnapshot, AntWorldRotater> pm(RenderSize);
        trainRoute(pm, routePath);

        size_t snapshot;
        float difference;
        std::vector<std::vector<float>> allDifferences;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(agent, 2_deg);
        LOGI << "Heading: " << heading;
        LOGI << "Best-matching snapshot: #" << snapshot;
        LOGI << "Difference score: " << difference;

        // Plot RIDF
        plotRIDF(allDifferences[snapshot]);
        LOGI;
    }

    {
        LOGI << "Using in silico rotater...";
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, BestMatchingSnapshot, InSilicoRotater> pm(RenderSize);
        trainRoute(pm, routePath);

        agent.setAttitude(0_deg, 0_deg, 0_deg);
        cv::Mat fr;
        agent.readGreyscaleFrame(fr);

        size_t snapshot;
        float difference;
        std::vector<std::vector<float>> allDifferences;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(fr);
        LOGI << "Heading: " << heading;
        LOGI << "Best-matching snapshot: #" << snapshot;
        LOGI << "Difference score: " << difference;

        // Plot RIDF
        plotRIDF(allDifferences[snapshot]);
        LOGI;
    }
}
