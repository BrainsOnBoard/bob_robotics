// BoB robotics includes
#include "common/timer.h"
#include "common/serialise_matrix.h"
#include "navigation/image_database.h"
#include "navigation/infomax.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/matplotlibcpp.h"
#include "third_party/path.h"
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <string>
#include <tuple>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
using namespace std::literals;
namespace plt = matplotlibcpp;

using FloatType = float;
using InfoMaxType = InfoMaxRotater<FloatType>;

void doTesting(const InfoMaxType &infomax, const std::vector<double> &x,
               const std::vector<double> &y, const std::vector<cv::Mat> &images)
{
    using namespace units::math;

    std::vector<double> u, v;
    {
        // Test network
        LOGI << "Testing network...";
        Timer<> t{ "Network testing took: " };
        for (const auto &image : images) {
            // Get heading and convert to vector
            const auto heading = std::get<0>(infomax.getHeading(image));
            u.push_back(cos(heading));
            v.push_back(sin(heading));
        }
    }

    // Output as quiver plot
    std::map<std::string, std::string> kwargs;
    kwargs["angles"] = "xy"; // This option is needed if you want matplotlib to do a sane plot :-/
    plt::quiver(x, y, u, v, kwargs);
    plt::xlabel("x (m)");
    plt::ylabel("y (m)");
    plt::plot({ x[0] }, { y[0] }, "b+");
    plt::show();
}

int bobMain(int argc, char **argv)
{
    if (argc < 2) {
        LOGE << "Must specify a route, e.g.:"
             << "\t" << argv[0] << " ../../tools/ant_world_db_creator/ant1_route1 [num training repititions]";
        return 1;
    }

    // Where we are loading images from
    const filesystem::path routePath(argv[1]);

    // We resize images to this
    const cv::Size imSize{ 90, 25 };

    // Load images, resize and cache
    std::vector<cv::Mat> images;
    std::vector<double> x, y;
    {
        LOGI << "Loading images from " << routePath << "...";
        Timer<> loadingTimer{ "Images loaded in: " };
        const ImageDatabase routeImages(routePath);
        assert(!routeImages.empty());

        cv::Mat image;
        for (size_t i = 1; i < routeImages.size(); i++) {
            image = routeImages[i].load();
            images.emplace_back(imSize, CV_8UC1);
            cv::resize(image, images.back(), imSize);

            // Save x and y in metres
            x.push_back(routeImages[i].position[0].value() / 1000.0);
            y.push_back(routeImages[i].position[1].value() / 1000.0);
        }
    }

    // Number of times each image is presented to network
    int numReps = (argc > 2) ? std::stoi(argv[2]) : 1;
    const filesystem::path netPath = "trained_network_"s + routePath.filename()
            + "_reps"s + std::to_string(numReps) + ".bin"s;

    // If we already have a network for these params, load from disk
    if (netPath.exists()) {
        LOGI << "Loading weights from " << netPath;
        const auto weights = readMatrix<FloatType>(netPath);

        InfoMaxType infomax(imSize, weights);
        doTesting(infomax, x, y, images);
    } else {
        // ...otherwise do the training now
        InfoMaxType infomax(imSize);

        // Train network
        {
            LOGI << "Training InfoMax network...";
            Timer<> trainingTimer{ "Network trained in: " };

            // We train the network with each image n times
            for (int i = 0; i < numReps; i++) {
                for (const auto &im : images) {
                    infomax.train(im);
                }
            }

            // Write weights to disk
            LOGI << "Writing weights to " << netPath;
            writeMatrix(netPath, infomax.getWeights());
        }

        doTesting(infomax, x, y, images);
    }

    return 0;
}
