// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"
#include "common/string.h"
#include "navigation/image_database.h"

// TBB
#include "tbb/parallel_for_each.h"

// Third-party includes
#include "third_party/CLI11.hpp"
#include "third_party/path.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

using namespace BoBRobotics;
using namespace Navigation;

// **TODO**: Allow user to choose output video format
constexpr const char *VideoExtension = ".mp4";
constexpr const char *VideoFormat = "mp4v";

bool
readCSVLine(std::ifstream &ifs, std::string &line,
            std::vector<std::string> &fields)
{
    if (!std::getline(ifs, line)) {
        return false;
    }

    strSplit(line, ',', fields);

    // Trim whitespace from fields
    std::for_each(fields.begin(), fields.end(), strTrim);

    return true;
}

// Strips out the Filename column from the csv file
void
convertCSVFile(const ImageDatabase &inDatabase,
               const filesystem::path &outPath)
{
    std::ifstream ifs{ (inDatabase.getPath() / ImageDatabase::EntriesFilename).str() };
    BOB_ASSERT(ifs.good());
    ifs.exceptions(std::ios::badbit);

    std::string line;
    std::vector<std::string> fields;

    BOB_ASSERT(readCSVLine(ifs, line, fields));
    const auto found = std::find(fields.cbegin(), fields.cend(), "Filename");
    const size_t filenameIdx = std::distance(fields.cbegin(), found);

    /*
     * Store a list of column indices excluding the one corresponding to
     * Filename. This step is necessary because otherwise it's fiddly to work
     * out where to write commas between fields.
     */
    std::vector<size_t> copyIdx;
    copyIdx.reserve(fields.size() - 1);
    for (size_t i = 0; i < fields.size() - 1; i++) {
        if (i != filenameIdx) {
            copyIdx.push_back(i);
        }
    }

    // Write out new CSV file without Filename column
    std::ofstream ofs{ (outPath / ImageDatabase::EntriesFilename).str() };
    BOB_ASSERT(ofs.good());
    ofs.exceptions(std::ios::badbit);
    do {
        ofs << fields[copyIdx[0]];
        for (size_t i = 1; i < copyIdx.size(); i++) {
            ofs << ", " << fields[copyIdx[i]];
        }
        ofs << "\n";
    } while (readCSVLine(ifs, line, fields));
}

void
convertYAMLFile(const ImageDatabase &inDatabase,
                const filesystem::path &outPath,
                const filesystem::path &videoFile,
                double frameRate)
{
    const auto outYamlPath = outPath / ImageDatabase::MetadataFilename;
    const auto inYamlPath = inDatabase.getPath() / ImageDatabase::MetadataFilename;
    if (inYamlPath.exists()) {
        filesystem::copy_file(inYamlPath, outYamlPath);
    } else {
        // If there's no metadata file then add one
        std::ofstream ofs{ outYamlPath.str() };
        BOB_ASSERT(ofs.good());
        ofs.exceptions(std::ios::badbit);
        ofs << R"(%YAML:1.0
---
metadata:
  type: route
  camera:
     name: pixpro_usb
     resolution: [ 1440, 1440 ]
     isPanoramic: 1
  needsUnwrapping: 1
  isGreyscale: 0)";
    }

    std::ofstream ofs{ outYamlPath.str(), std::ios::app };
    BOB_ASSERT(ofs.good());
    ofs.exceptions(std::ios::badbit);
    ofs << "\n  videoFile: " << videoFile.filename() << "\n"
        << "  frameRate: " << frameRate << "\n"
        << "  image_database_to_video_git_commit: " BOB_ROBOTICS_GIT_COMMIT "\n";
}

void
writeVideoFile(const ImageDatabase &inDatabase,
               const filesystem::path &videoFile,
               double frameRate)
{
    const auto imageSize = inDatabase[0].load(false).size();

    cv::VideoWriter writer{ videoFile.str(),
                            cv::VideoWriter::fourcc(VideoFormat[0], VideoFormat[1], VideoFormat[2], VideoFormat[3]),
                            frameRate,
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
    double frameRate;

    CLI::App app{ "Tool for converting image databases composed of separate image files to video-type databases (i.e. to save space)." };
    app.add_option("-o,--output-dir", outputDir, "Folder to save converted databases into");
    app.add_option("-f", frameRate, "Frame rate at which image sequence was recorded")->required();

    app.allow_extras();
    CLI11_PARSE(app, argc, argv);
    if (app.remaining_size() == 0) {
        std::cout << app.help();
        return EXIT_FAILURE;
    }

    if (!outputDir.exists()) {
        BOB_ASSERT(filesystem::create_directory(outputDir));
    }

    const auto convertDatabase = [&](const auto &databasePath) {
        const ImageDatabase inDatabase{ databasePath };
        BOB_ASSERT(!inDatabase.isVideoType());
        BOB_ASSERT(!inDatabase.empty());

        const auto outPath = outputDir / inDatabase.getName();
        // **TODO**: Allow for optionally overwriting converted databases
        BOB_ASSERT(!outPath.exists());
        LOGI << "Saving new database to " << outPath;
        filesystem::create_directory(outPath);

        const auto videoFile = outPath / (inDatabase.getName() + VideoExtension);
        convertCSVFile(inDatabase, outPath);
        convertYAMLFile(inDatabase, outPath, videoFile, frameRate);
        writeVideoFile(inDatabase, videoFile, frameRate);
    };

    // Convert databases in parallel
    tbb::parallel_for_each(app.remaining(), convertDatabase);

    return EXIT_SUCCESS;
}
