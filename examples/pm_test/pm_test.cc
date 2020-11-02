// BoB robotics includes
#include "common/timer.h"
#include "imgproc/mask.h"
#include "imgproc/pipeline.h"
#include "imgproc/roll_image.h"
#include "navigation/algorithms.h"
#include "navigation/image_database.h"
#include "navigation/perfect_memory_new.h"
#include "viz/plot_ridf.h"
#include "common/range.h"
#include "common/path.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/path.h"
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;

int
bobMain(int, char **argv)
{
    const cv::Size imSize{ 180, 50 };
    const Navigation::ImageDatabase database{ Path::getRepoPath() / "tools" / "ant_world_db_creator" / "ant1_route1" };
    const auto snapshots = database.loadImages(CV_32F, imSize);

    {
        LOGI << "Testing with best-matching snapshot method...";

        // Default algorithm: find best-matching snapshot, use abs diff
        Navigation::PerfectMemory<cv::Mat, cv::Size> pm(imSize);
        for (const auto &snapshot : snapshots) {
            pm.train(snapshot);
        }

        for (int i = 0; i < 10; i++) {
            // Time testing phase
            Timer<> t{ "Time taken for testing: " };

            // Treat snapshot #10 as test data
            const auto &image = snapshots[10];
            size_t snapshot;
            float difference;
            degree_t heading;
            std::tie(heading, snapshot, difference) = pm.getHeadingAndSnapshotInfo(image);
            LOGI << "Heading: " << heading;
            LOGI << "Best-matching snapshot: #" << snapshot;
            LOGI << "Difference score: " << difference;
        }
    }

    return EXIT_SUCCESS;
}
