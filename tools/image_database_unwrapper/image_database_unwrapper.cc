// BoB robotics includes
#include "navigation/image_database.h"

// Third-party includes
#include "third_party/CLI11.hpp"

// Standard C++ includes
#include <string>

using namespace BoBRobotics;

int bobMain(int argc, char **argv)
{
    std::vector<size_t> size{ 720, 150 };
    size_t frameSkip = 1;
    bool greyscale = false;

    CLI::App app{ "Tool for unwrapping image databases." };
    app.allow_extras();
    app.add_option("-s,--skip-frames", frameSkip, "Number of frames to skip");
    auto opt = app.add_option("-r,--resolution", size, "Resolution of unwrapped images");
    opt->expected(2);
    app.add_flag("-g,--greyscale", greyscale, "Convert images to greyscale");
    CLI11_PARSE(app, argc, argv);
    if (app.remaining_size() != 1) {
        std::cout << app.help();
        return EXIT_FAILURE;
    }

    // Unwrap image database
    const filesystem::path inPath{ app.remaining()[0] };
    const Navigation::ImageDatabase database(inPath);
    const filesystem::path outPath = inPath.parent_path() /
                                        ("unwrapped_" + inPath.filename());
    std::cout << "Creating new database in " << outPath << "\n";
    database.unwrap(outPath, { (int) size[0], (int) size[1] }, frameSkip, greyscale);

    return EXIT_SUCCESS;
}
