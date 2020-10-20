// BoB robotics includes
#include "imgproc/mask.h"
#include "imgproc/pipeline.h"
#include "imgproc/roll_image.h"
#include "navigation/algorithms.h"
#include "navigation/image_database.h"
#include "navigation/perfect_memory_new.h"
#include "viz/plot_ridf.h"

// Third-party includes
#include "third_party/path.h"

using namespace BoBRobotics;

int bobMain(int, char** argv)
{
    const auto programFolder = filesystem::path{ argv[0] }.parent_path();
    const ImgProc::Mask<> mask{ programFolder / "mask.png" };
    const Navigation::ImageDatabase database{ programFolder / "../../tools" / "ant_world_db_creator" / "ant1_route1" };
    const auto snapshots = database.loadImages(CV_32F, mask.size());

    Navigation::PerfectMemory<cv::Mat, cv::Size> pm{ mask.size() };
    for (const auto &snapshot : snapshots) {
        pm.train(snapshot);
    }

    const auto &image = snapshots[10];
    const auto &diffs = Navigation::calculateRIDF(image, snapshots[10]);
    // cv::imshow("image", snapshots[10]);
    // cv::waitKey(0);

    // cv::imshow("snap", snapshots[9]);
    // cv::waitKey(0);

    Viz::plotRIDF(diffs);

    return EXIT_SUCCESS;
}

// int bobMain(int, char**)
// {
//     const ImgProc::Mask<> mask{ "mask.png" };
//     const auto pl = ImgProc::createPipeline(mask);

//     std::cout << mask.size() << "\n";
//     auto image = cv::imread("../../tools/ant_world_db_creator/ant1_route1/image_00000.png", cv::IMREAD_GRAYSCALE);
//     cv::resize(image, image, mask.size());
//     const auto &maskedImage = pl(image);
//     const auto restored = mask.reconstructImage(maskedImage);
//     cv::imshow("image!", restored);
//     cv::waitKey(0);

//     return EXIT_SUCCESS;
// }
