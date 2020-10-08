// BoB robotics includes
#include "navigation/perfect_memory.h"
#include "viz/plot_ridf.h"

using namespace BoBRobotics::Navigation;
using namespace BoBRobotics::Viz;

int bobMain(int, char **)
{
    // Class to run perfect memory algorithm
    cv::Size imSize(180, 50);
    PerfectMemoryRotater<> pm(imSize);

    // Load a single snapshot
    cv::Mat snap = cv::imread("../../tools/ant_world_db_creator/ant1_route1/image_00010.png", cv::IMREAD_GRAYSCALE);
    cv::resize(snap, snap, imSize);
    pm.train(snap);

    // Compare snapshot with itself
    const auto differences = pm.getImageDifferences(snap).row(0);

    // Plot RIDF
    std::vector<float> diffVec;
    for (int i = 0; i < differences.cols(); i++) {
        diffVec.push_back(differences(i));
    }
    plotRIDF(diffVec);

    return EXIT_SUCCESS;
}
