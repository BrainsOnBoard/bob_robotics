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
using namespace units::angle;
using namespace units::length;
using namespace units::math;
namespace plt = matplotlibcpp;

using FloatType = float;
using InfoMaxType = InfoMaxRotater<FloatType>;

void doTesting(const InfoMaxType &infomax, const ImageDatabase &route,
               const std::vector<cv::Mat> &images)
{
    using namespace units::math;
    constexpr size_t TestSkip = 20;

    std::vector<double> x, y, xAll, yAll, u, v;
    std::vector<radian_t> thetaAll;
    for (size_t i = 0; i < route.size() - 1; i++) {
        const Vector3<meter_t> pos = route[i].pose;
        xAll.push_back(pos.x().value());
        yAll.push_back(pos.y().value());
        thetaAll.push_back(atan2(route[i + 1].pose.y() - pos.y(),
                                 route[i + 1].pose.x() - pos.x()));
    }
    const Vector3<meter_t> pos = route.getEntries().back().pose;
    xAll.push_back(pos.x().value());
    yAll.push_back(pos.y().value());
    thetaAll.push_back(thetaAll.back());

    {
        // Test network
        LOGI << "Testing network...";
        Timer<> t{ "Network testing took: " };
        for (size_t i = 0; i < route.size(); i += TestSkip) {
            const Pose3<meter_t, radian_t> pose = route[i].pose;
            x.push_back(pose.x().value());
            y.push_back(pose.y().value());

            // Get heading and convert to vector
            const radian_t heading = std::get<0>(infomax.getHeading(images[i])) + thetaAll[i];
            u.push_back(cos(heading));
            v.push_back(sin(heading));
        }
    }

    // Output as quiver plot
    std::map<std::string, std::string> kwargs;
    kwargs["angles"] = "xy"; // This option is needed if you want matplotlib to do a sane plot :-/
    kwargs["scale_units"] = "xy";
    plt::plot(x, y);
    plt::quiver(x, y, u, v, kwargs);
    plt::xlabel("x (m)");
    plt::ylabel("y (m)");
    plt::plot({ x[0] }, { y[0] }, "b+");
    plt::axis("equal");
    plt::show();
}

int bobMain(int argc, char **argv)
{
    if (argc < 2) {
        LOGE << "Must specify a route, e.g.:"
             << "\t" << argv[0] << " ../../tools/ant_world_db_creator/ant1_route1 [num training repetitions]";
        return 1;
    }

    // Where we are loading images from
    const filesystem::path routePath(argv[1]);

    // We resize images to this
    const cv::Size imSize{ 90, 25 };

    // Load images, resize and cache
    std::vector<cv::Mat> images;
    const ImageDatabase route(routePath);
    assert(!route.empty());
    {
        LOGI << "Loading images from " << routePath << "...";
        Timer<> loadingTimer{ "Images loaded in: " };

        cv::Mat image;
        for (size_t i = 1; i < route.size(); i++) {
            image = route[i].load();
            images.emplace_back(imSize, CV_8UC1);
            cv::resize(image, images.back(), imSize);
        }
    }

    // Number of times each image is presented to network
    int numReps = (argc > 2) ? std::stoi(argv[2]) : 1;
    const filesystem::path netPath = "trained_network_"s + routePath.filename()
            + "_reps"s + std::to_string(numReps) + ".bin"s;

    constexpr auto LearningRate = InfoMaxType::DefaultLearningRate;

    // If we already have a network for these params, load from disk
    if (netPath.exists()) {
        LOGI << "Loading weights from " << netPath;

        InfoMaxType infomax(imSize, LearningRate, Normalisation::None, 
                            readMatrix<FloatType>(netPath));
        doTesting(infomax, route, images);
    } else {
        // ...otherwise do the training now
        InfoMaxType infomax(imSize, LearningRate);

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

        doTesting(infomax, route, images);
    }

    return 0;
}
