// BoB robotics includes
#include "navigation/image_database.h"

// Standard C++ includes
#include <string>

using namespace BoBRobotics;

int bobMain(int argc, char **argv)
{
    // We must have a path + an optional image size
    BOB_ASSERT(argc == 2 || argc == 4);

    // Either parse image size from command line or use default
    const cv::Size unwrapRes = argc == 4 ? cv::Size(std::stoi(argv[2]), std::stoi(argv[3]))
                                         : cv::Size(720, 150);

    // Unwrap image database
    const filesystem::path inPath{ argv[1] };
    const Navigation::ImageDatabase database(inPath);
    const filesystem::path outPath = inPath.parent_path() /
                                        ("unwrapped_" + inPath.filename());
    std::cout << "Creating new database in " << outPath << "\n";
    database.unwrap(outPath, unwrapRes);

    return EXIT_SUCCESS;
}
