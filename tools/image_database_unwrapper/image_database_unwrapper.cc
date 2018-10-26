// BoB robotics includes
#include "navigation/image_database.h"

// Standard C++ includes
#include <string>

using namespace BoBRobotics;

int
main(int argc, char **argv)
{
    // We must have a path + an optional image size
    BOB_ASSERT(argc == 2 || argc == 4);

    // Either parse image size from command line or use default
    const cv::Size unwrapRes = argc == 4 ? cv::Size(std::stoi(argv[2]), std::stoi(argv[3]))
                                         : cv::Size(720, 58);

    // Unwrap image database
    Navigation::ImageDatabase database(argv[1]);
    database.unwrap("unwrapped_" + database.getName(), unwrapRes);
}