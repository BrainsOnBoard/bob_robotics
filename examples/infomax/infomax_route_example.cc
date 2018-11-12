#include "read_data.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <string>
#include <tuple>

// Eigen
#include <Eigen/Core>

// OpenCV
#include "opencv2/opencv.hpp"

// BoB robotics includes
#include "common/timer.h"
#include "navigation/image_database.h"
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"
#include "third_party/path.h"
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
namespace plt = matplotlibcpp;

using namespace units::literals;
using namespace units::math;

using FloatType = float;
using InfoMaxType = InfoMaxRotater<InSilicoRotater, FloatType>;

void doTesting(const InfoMaxType &infomax, const std::vector<double> &allx,
                 const std::vector<double> &ally, const std::vector<cv::Mat> &images)
{
    std::vector<double> x, y, u, v;
    {
        // Test network
        std::cout << "Testing network..." << std::endl;
        Timer<> t{ "Network testing took: " };
        for (size_t i = 0; i < images.size(); i++) {
            x.push_back(allx[i]);
            y.push_back(ally[i]);

            // Get heading and convert to vector
            const units::angle::radian_t heading = std::get<0>(infomax.getHeading(images[i]));
            double sinx, cosx;
            sincos(heading.value(), &sinx, &cosx);
            u.push_back(cosx);
            v.push_back(sinx);
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

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "Error: must specify a route, e.g.:" << std::endl
                  << "\t" << argv[0] << " ../../tools/ant_world_db_creator/ant1_route1 [num training repititions]" << std::endl;
        return 1;
    }

    std::cout << "Eigen is using " << Eigen::nbThreads() << " threads." << std::endl
              << std::endl;

    // Where we are loading images from
    const filesystem::path routePath(argv[1]);

    // We resize images to this
    const cv::Size imSize{ 90, 25 };

    // Load images, resize and cache
    std::vector<cv::Mat> images;
    std::vector<double> x, y;
    {
        std::cout << "Loading images from " << routePath << "..." << std::endl;
        Timer<> loadingTimer{ "Images loaded in: " };
        const ImageDatabase routeImages(routePath);
        assert(routeImages.size() > 0);

        cv::Mat image;
        for (size_t i = 1; i < routeImages.size(); i++) {
            image = routeImages[i].loadGreyscale();
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
        std::cout << "Loading weights from " << netPath << std::endl;
        const auto weights = readData<FloatType>(netPath);

        InfoMaxType infomax(imSize, weights);
        doTesting(infomax, x, y, images);
    } else {
        // ...otherwise do the training now
        InfoMaxType infomax(imSize);

        // Train network
        {
            std::cout << "Training InfoMax network..." << std::endl;
            Timer<> trainingTimer{ "Network trained in: " };

            // We train the network with each image n times
            for (int i = 0; i < numReps; i++) {
                for (const auto &im : images) {
                    infomax.train(im);
                }
            }

            // Write weights to disk
            std::cout << "Writing weights to " << netPath << std::endl;
            const auto weights = infomax.getWeights();
            std::ofstream netFile(netPath.str(), std::ios::binary);
            const int size[2] { (int) weights.rows(), (int) weights.cols() };
            netFile.write(reinterpret_cast<const char *>(size), sizeof(size));
            netFile.write(reinterpret_cast<const char *>(weights.data()), weights.size() * sizeof(float));
        }

        doTesting(infomax, x, y, images);
    }

    return 0;
}
