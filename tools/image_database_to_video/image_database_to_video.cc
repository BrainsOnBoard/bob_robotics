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
#include <sstream>
#include <string>

using namespace BoBRobotics;
using namespace Navigation;

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

bool
convertYAMLFile(const ImageDatabase &inDatabase,
                const filesystem::path &outPath,
                const filesystem::path &videoFile,
                double frameRate,
                const std::string &defaultMetadata)
{
    const auto outYamlPath = outPath / ImageDatabase::MetadataFilename;
    const auto inYamlPath = inDatabase.getPath() / ImageDatabase::MetadataFilename;
    if (inYamlPath.exists()) {
        filesystem::copy_file(inYamlPath, outYamlPath);
    } else if (!defaultMetadata.empty()) {
        // If there's no metadata file then add one
        std::ofstream ofs{ outYamlPath.str() };
        BOB_ASSERT(ofs.good());
        ofs.exceptions(std::ios::badbit);
        ofs << defaultMetadata;
    } else {
        LOGE << "No metadata found and default metadata file was not specified. Skipping "
             << inDatabase.getPath();
        return false;
    }

    std::ofstream ofs{ outYamlPath.str(), std::ios::app };
    BOB_ASSERT(ofs.good());
    ofs.exceptions(std::ios::badbit);
    ofs << "\n  videoFile: '" << videoFile.filename() << "'\n"
        << "  frameRate: " << frameRate << "\n"
        << "  image_database_to_video_git_commit: '" BOB_ROBOTICS_GIT_COMMIT "'\n";
    return true;
}

void
writeVideoFile(const ImageDatabase &inDatabase,
               const filesystem::path &videoFile,
               double frameRate,
               const std::string &fourcc)
{
    const auto imageSize = inDatabase[0].load(false).size();

    cv::VideoWriter writer{ videoFile.str(),
                            cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]),
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
    filesystem::path defaultMetadataPath;
    std::string fileExtension = "avi", fourcc = "MJPG";
    double frameRate;

    CLI::App app{ "Tool for converting image databases composed of separate image files to video-type databases (i.e. to save space)." };
    app.add_option("-o,--output-dir", outputDir, "Folder to save converted databases into");
    app.add_option("-d,--default-metadata", defaultMetadataPath, "Path to a default metadata file to use if one is not present");
    app.add_option("-r", frameRate, "Frame rate at which image sequence was recorded")->required();
    app.add_option("-f,--format", fileExtension, "Extension for video file (default: avi)");
    app.add_option("--fourcc", fourcc, "Fourcc code for video stream (default: MJPG)");

    app.allow_extras();
    CLI11_PARSE(app, argc, argv);
    if (app.remaining_size() == 0) {
        std::cout << app.help();
        return EXIT_FAILURE;
    }

    BOB_ASSERT(fourcc.size() == 4);

    if (!outputDir.exists()) {
        BOB_ASSERT(filesystem::create_directory(outputDir));
    }

    // Read in default metadata YAML to use if none is present
    std::string defaultMetadata;
    if (!defaultMetadataPath.empty()) {
        std::ifstream ifs{ defaultMetadataPath.str() };
        std::stringstream ss;
        ss << ifs.rdbuf();
        defaultMetadata = ss.str();
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

        const auto videoFile = outPath / (inDatabase.getName() + "." + fileExtension);
        if (convertYAMLFile(inDatabase, outPath, videoFile, frameRate, defaultMetadata)) {
            convertCSVFile(inDatabase, outPath);
            writeVideoFile(inDatabase, videoFile, frameRate, fourcc);
        }
    };

    // Convert databases in parallel
    tbb::parallel_for_each(app.remaining(), convertDatabase);

    return EXIT_SUCCESS;
}
