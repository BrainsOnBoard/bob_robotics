// BoB robotics includes
#include "common/pose.h"
#include "robots/simulated_tank.h"
#include "viz/agent_renderer.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::length;

auto
loadObjects(const filesystem::path &objectsPath)
{
    std::vector<std::vector<Vector2<millimeter_t>>> objects;

    std::cout << "Loading object positions from " << objectsPath << "..." << std::endl;
    cv::FileStorage fs(objectsPath.str(), cv::FileStorage::READ);
    std::vector<double> vertex(2);
    for (auto objectNode : fs["objects"]) {
        objects.emplace_back();
        for (auto vertexNode : objectNode) {
            vertexNode >> vertex;
            objects.back().emplace_back(Vector2<millimeter_t>{ millimeter_t{ vertex[0] },
                                                               millimeter_t{ vertex[1] } });
        }
    }

    return objects;
}

int
main(int, char **argv)
{
    const auto objectsPath = filesystem::path(argv[0]).parent_path() / "objects.yaml";
    const auto objects = loadObjects(objectsPath);

    Viz::AgentRenderer<> renderer;
    renderer.addObjects(objects);

    // run the program as long as the window is open
    while (renderer.isOpen()) {
        renderer.update();
        std::this_thread::sleep_for(5ms);
    }
}
