// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"
#include "robots/gantry/gantry.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <array>
#include <iostream>

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::length;

std::array<millimeter_t, 3>
getPosition(const cv::FileNode &node)
{
    std::vector<double> vec;
    node >> vec;
    BOB_ASSERT(vec.size() == 3);

    std::array<millimeter_t, 3> pos{
        millimeter_t{ vec[0] },
        millimeter_t{ vec[1] },
        millimeter_t{ vec[2] }
    };
    for (size_t i = 0; i < 3; i++) {
        BOB_ASSERT(pos[i] >= 0_mm && pos[i] < Robots::Gantry::Limits[i]);
    }

    return pos;
}

int
bobMain(int, char **)
{
    // Object to interface with gantry robot
    Robots::Gantry gantry;

    // Find config file. If there's not one in the same folder as the program, use the example one from the repo.
    auto configFilePath = Path::getProgramPath() / "gantry_dvs_config.yaml";
    if (!configFilePath.exists()) {
        configFilePath = Path::getRepoPath() / "projects" / "gantry_dvs" / "gantry_dvs_config.yaml";
    }
    BOB_ASSERT(configFilePath.exists());

    // Parse config file
    cv::FileStorage fs{ configFilePath.str(), cv::FileStorage::READ };
    const auto node = fs["config"];
    const auto startPos = getPosition(node["start"]);
    const auto endPos = getPosition(node["end"]);

    // Move gantry to specified locations
    std::cout << "Homing gantry...\n";
    gantry.raiseAndHome();
    std::cout << "Gantry homed.\nMoving to start position...\n";
    gantry.setPosition(startPos[0], startPos[1], startPos[2]);
    gantry.waitToStopMoving();
    std::cout << "Ready! Press any key to start moving.\n";
    std::cin.ignore();
    gantry.setPosition(endPos[0], endPos[1], endPos[2]);
    gantry.waitToStopMoving();
    std::cout << "Finished!";

    return EXIT_SUCCESS;
}
