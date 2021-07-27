// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"
#include "navigation/image_database.h"

// TBB
#include "tbb/parallel_for_each.h"

// Third-party includes
#include "third_party/CLI11.hpp"
#include "third_party/path.h"

// Standard C++ includes
#include <iostream>
#include <mutex>
#include <string>

using namespace BoBRobotics;

// **TODO**: Allow user to choose output video format
constexpr const char *VideoExtension = ".mp4";
constexpr const char *VideoFormat = "mp4v";

void
writeVideoFile(const Navigation::ImageDatabase &inDatabase,
               const filesystem::path &outPath)
{
    const auto imageSize = inDatabase[0].load(false).size();

    // **TODO**: Set FPS correctly
    const auto videoPath = outPath / (inDatabase.getName() + VideoExtension);
    cv::VideoWriter writer{ videoPath.str(),
                            cv::VideoWriter::fourcc(VideoFormat[0], VideoFormat[1], VideoFormat[2], VideoFormat[3]),
                            30.0, /*fps*/
                            imageSize };
    BOB_ASSERT(writer.isOpened());

    for (const auto &entry : inDatabase) {
        writer.write(entry.load(/*greyscale=*/false));
    }
}

int
bobMain(int argc, char **argv)
{
    filesystem::path outputDir = "video_databases";

    CLI::App app{ "Tool for converting image databases composed of separate image files to video-type databases (i.e. to save space)." };
    // **TODO**: Add option to delete converted databases
    app.add_option("-o,--output-dir", outputDir, "Folder to save converted databases into");
    app.allow_extras();
    CLI11_PARSE(app, argc, argv);
    if (app.remaining_size() == 0) {
        std::cout << app.help();
        return EXIT_FAILURE;
    }

    if (!outputDir.exists()) {
        BOB_ASSERT(filesystem::create_directory(outputDir));
    }

    std::mutex printMtx;
    const auto convertDatabase = [&](const auto &databasePath) {
        const Navigation::ImageDatabase inDatabase{ databasePath };
        BOB_ASSERT(!inDatabase.isVideoType());
        BOB_ASSERT(!inDatabase.empty());

        const auto outPath = outputDir / inDatabase.getName();
        // **TODO**: Allow for optionally overwriting converted databases
        BOB_ASSERT(!outPath.exists());
        printMtx.lock();
        std::cout << "Saving new database to " << outPath << "...\n";
        printMtx.unlock();
        filesystem::create_directory(outPath);

        writeVideoFile(inDatabase, outPath);
    };

    // Convert databases in parallel
    tbb::parallel_for_each(app.remaining(), convertDatabase);

    return EXIT_SUCCESS;
}
