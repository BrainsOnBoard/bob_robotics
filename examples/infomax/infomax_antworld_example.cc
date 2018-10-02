// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <set>
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

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
namespace plt = matplotlibcpp;

int
main()
{
    const cv::Size imSize{ 90, 25 };
    const ImageDatabase routeImages("../../tools/ant_world_db_creator/ant1_route1");

    std::cout << "Eigen is using " << Eigen::nbThreads() << " threads." << std::endl
              << std::endl;

    // Object to run InfoMax algorithm
    InfoMax<> infomax(imSize);

    // Train network
    {
        std::cout << "Training InfoMax network..." << std::endl;
        Timer<> trainingTimer{ "Network trained in: " };
        infomax.trainRoute(routeImages, true);
    }

    const ImageDatabase testImages("../../tools/ant_world_db_creator/world5000_grid");
    std::vector<double> x, y, u, v;
    std::set<millimeter_t> xset, yset;
    std::vector<millimeter_t> xunique, yunique;
    for (const auto &e : testImages) {
        xset.insert(e.position[0]);
        yset.insert(e.position[1]);
    }
    std::copy(xset.begin(), xset.end(), std::back_inserter(xunique));
    std::copy(yset.begin(), yset.end(), std::back_inserter(yunique));

    {
        // Test network
        std::cout << "Testing network..." << std::endl;
        Timer<> t{ "Network testing took: " };
        for (size_t i = 0; i < xunique.size(); i += 20) {
            for (size_t j = 0; j < yunique.size(); j += 20) {
                auto curx = xunique[i];
                auto cury = yunique[j];
                const auto result = std::find_if(testImages.begin(), testImages.end(),
                    [curx, cury](const auto &e) {
                        return e.position[0] == curx && e.position[1] == cury;
                    });

                if (result != testImages.end()) {
                    x.push_back(curx.value() / 1000.0);
                    y.push_back(cury.value() / 1000.0);

                    cv::Mat im = result->loadGreyscale();
                    cv::resize(im, im, imSize);
                    const radian_t heading = std::get<0>(infomax.getHeading(im));
                    double sinx, cosx;
                    sincos(heading.value(), &sinx, &cosx);
                    u.push_back(sinx);
                    v.push_back(cosx);
                }
            }
        }
    }

    // Output as quiver plot
    plt::quiver(x, y, u, v);
    plt::show();

    return 0;
}