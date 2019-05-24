// BoB robotics includes
#include "navigation/perfect_memory.h"
#include "navigation/plot.h"

using namespace BoBRobotics::Navigation;

int
main()
{
    // Class to run perfect memory algorithm
    cv::Size imSize(180, 50);
    PerfectMemoryRotater<> pm(imSize);

    // Load a single snapshot
    cv::Mat snap = cv::imread("../../tools/ant_world_db_creator/ant1_route1/image_00010.png", cv::IMREAD_GRAYSCALE);
    cv::resize(snap, snap, imSize);
    pm.train(snap);

    // Compare snapshot with itself
    std::vector<float> differences = pm.getImageDifferences(snap)[0];

    // Plot RIDF
    plotRIDF(differences);
}