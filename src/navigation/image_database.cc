// BoB robotics includes
#include "common/macros.h"
#include "common/string.h"
#include "imgproc/opencv_unwrap_360.h"
#include "navigation/image_database.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/tinydir.h"

// Standard C includes
#include <cstring>
#include <ctime>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

using namespace units::literals;
using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace Navigation {

constexpr const char *ImageDatabase::MetadataFilename;
constexpr const char *ImageDatabase::EntriesFilename;

size_t
Range::size() const
{
    return (separation == 0_mm) ? 1 : (1 + ((end - begin) / separation).to<size_t>());
}

cv::Mat
ImageDatabase::Entry::load(bool greyscale) const
{
    auto img = cv::imread(path.str(), greyscale ? cv::IMREAD_GRAYSCALE : cv::IMREAD_COLOR);
    BOB_ASSERT(!img.empty());
    return img;
}

bool
ImageDatabase::Entry::hasExtraField(const std::string &name) const
{
    return extraFields.find(name) != extraFields.cend();
}

const std::string &
ImageDatabase::Entry::getExtraField(const std::string &name) const
{
    const auto pos = extraFields.find(name);
    BOB_ASSERT(pos != extraFields.cend());

    // This is needed because operator[] is non-const for std::unordered_map
    return pos->second;
}

void
ImageDatabase::ImageFileWriter::setImageFormat(std::string format)
{
    m_ImageFormat = std::move(format);
}

void
ImageDatabase::ImageFileWriter::writeFrame(const cv::Mat &frame, Entry &entry)
{
    filesystem::path path = getCurrentFilenameRoot() + "." + m_ImageFormat;
    BOB_ASSERT(!path.exists()); // Don't overwrite data by default!
    BOB_ASSERT(cv::imwrite(path.str(), frame));

    entry.path = std::move(path);
}

ImageDatabase::VideoFileWriter::VideoFileWriter(const ImageDatabase &database)
  : m_FileName{ database.getName() + ".mp4" }
{
    const auto path = database.getPath() / m_FileName;
    BOB_ASSERT(!path.exists()); // Don't overwrite by mistake
    m_Writer.open(path.str(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                  database.getFrameRate().value(), database.getResolution());
    BOB_ASSERT(m_Writer.isOpened());
}

const std::string &
ImageDatabase::VideoFileWriter::getVideoFileName() const
{
    return m_FileName;
}

void
ImageDatabase::VideoFileWriter::writeFrame(const cv::Mat &frame, Entry &)
{
    m_Writer.write(frame);
}

std::tm
getCurrentTime()
{
    auto timer = time(nullptr);
    return *std::localtime(&timer);
}

ImageDatabase::ImageDatabase()
  : ImageDatabase{ getCurrentTime() }
{
}

ImageDatabase::ImageDatabase(const std::tm &creationTime)
  : ImageDatabase{ &creationTime, Path::getNewPath(creationTime), false }
{
}

ImageDatabase::ImageDatabase(const char *databasePath, bool overwrite)
  : ImageDatabase(filesystem::path(databasePath), overwrite)
{}

ImageDatabase::ImageDatabase(const std::string &databasePath, bool overwrite)
  : ImageDatabase(filesystem::path(databasePath), overwrite)
{}

ImageDatabase::ImageDatabase(filesystem::path databasePath, bool overwrite)
  : ImageDatabase{ nullptr, std::move(databasePath), overwrite }
{
}

ImageDatabase::ImageDatabase(const std::tm *creationTime,
                             filesystem::path databasePath,
                             bool overwrite)
  : m_Path{ std::move(databasePath) }
{
    memset(&m_CreationTime, 0, sizeof(m_CreationTime));
    m_CreationTime.tm_isdst = -1;

    LOGI << "Using image database at " << m_Path;

    // If we're making a new database then we need a creation time
    if (overwrite && m_Path.exists()) {
        m_CreationTime = creationTime ? *creationTime : getCurrentTime();

        LOG_WARNING << "Database already exists; overwriting";
        filesystem::remove_all(m_Path);
        BOB_ASSERT(filesystem::create_directory(m_Path));
        return;
    }

    // Allow for loading a standalone video file as an image database
    if (m_Path.is_file()) {
        auto ext = m_Path.extension();
        for (char &c : ext) {
            c = ::tolower(c);
        }
        BOB_ASSERT(ext == "mp4");

        m_VideoFilePath = m_Path;
        m_Path = m_Path.parent_path();

        // Populate m_Entries with empty entries
        cv::VideoCapture cap{ m_VideoFilePath.str() };
        BOB_ASSERT(cap.isOpened());
        m_Entries.resize(static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_COUNT)));
    } else {
        BOB_ASSERT(m_Path.is_directory());
    }

    // Try to read metadata from YAML file
    loadMetadata();

    // Try to read entries from CSV file
    if (!loadCSV()) {
        LOGW << "Could not find CSV file";
        if (m_VideoFilePath.empty() && !readDirectoryEntries()) {
            // Make sure we have a directory to save into
            filesystem::create_directory(m_Path);

            /*
             * Now that we *know* it's an empty database, it makes sense to
             * assign a creation time.
             */
            m_CreationTime = creationTime ? *creationTime : getCurrentTime();
        }
    }
}

bool
ImageDatabase::loadCSV()
{
    const auto entriesPath = m_Path / EntriesFilename;
    std::ifstream entriesFile(entriesPath.str());
    if (entriesFile.fail()) {
        return false;
    }
    entriesFile.exceptions(std::ios::badbit);

    std::string line;
    if (!std::getline(entriesFile, line)) {
        // ...then it's an empty file
        return false;
    }

    // Read field names, using comma as separator and trimming whitespace
    std::vector<std::string> fields;
    strSplit(line, ',', fields);
    std::for_each(fields.begin(), fields.end(), strTrim);
    const size_t numFields = fields.size();

    constexpr std::array<const char *, 8> defaultFieldNames{
        "X [mm]", "Y [mm]", "Z [mm]", "Heading [degrees]", "Filename", "Grid X",
        "Grid Y", "Grid Z"
    };

    /*
     * Go through field names, figuring out which are standard ones and which
     * are extra, user-defined ones
     */
    std::array<int, defaultFieldNames.size()> fieldNameIdx;
    std::fill(fieldNameIdx.begin(), fieldNameIdx.end(), -1);
    std::unordered_map<size_t, std::string> extraFieldNames;
    for (size_t i = 0; i < fields.size(); i++) {
        const auto def = std::find(defaultFieldNames.begin(), defaultFieldNames.end(), fields[i]);
        if (def != defaultFieldNames.end()) {
            // If it's a default one, save the column number for later parsing
            fieldNameIdx[std::distance(defaultFieldNames.begin(), def)] = i;
        } else {
            extraFieldNames.emplace(i, std::move(fields[i]));
        }
    }

    // Sanity check the file: we need the first five columns
    const auto validIdx = [](int idx) {
        return idx != -1;
    };
    BOB_ASSERT(std::all_of(fieldNameIdx.cbegin(), fieldNameIdx.cbegin() + 4, validIdx));

    /*
     * The Filename column is optional, but only if there is a video file
     * associated with this database.
     */
    if (!validIdx(fieldNameIdx[4])) {
        BOB_ASSERT(!m_VideoFilePath.empty());
    }

    // Assume it's a grid if we have any of the Grid fields...
    if (!hasMetadata()) {
        m_IsRoute = !std::any_of(fieldNameIdx.cbegin() + 5, fieldNameIdx.cend(), validIdx);
    }

    // ...but we require *all* Grid fields for it to be valid
    if (!m_IsRoute) {
        BOB_ASSERT(std::all_of(fieldNameIdx.cbegin() + 5, fieldNameIdx.cend(), validIdx));
    }

    // Read data line by line
    while (std::getline(entriesFile, line)) {
        // Ignore empty lines
        strTrim(line);
        if (line.empty()) {
            continue;
        }

        // Use comma as delimiter
        strSplit(line, ',', fields);
        BOB_ASSERT(fields.size() == numFields);
        std::for_each(fields.begin(), fields.end(), strTrim);

        // Get values for any user-defined fields
        std::unordered_map<std::string, std::string> extraFields;
        for (size_t i = 0; i < fields.size(); i++) {
            const auto pos = extraFieldNames.find(i);
            if (pos != extraFieldNames.cend()) {
                extraFields.emplace(pos->second, std::move(fields[i]));
            }
        }

        // Get the value for a given default field
        const auto getDefaultField = [&fields, &fieldNameIdx](size_t i) {
            return fields[fieldNameIdx[i]];
        };

        // Get grid position for grid databases
        std::array<size_t, 3> gridPosition;
        if (!m_IsRoute) {
            gridPosition[0] = static_cast<size_t>(std::stoul(getDefaultField(5)));
            gridPosition[1] = static_cast<size_t>(std::stoul(getDefaultField(6)));
            gridPosition[2] = static_cast<size_t>(std::stoul(getDefaultField(7)));
        }

        // Save details to vector
        Entry entry{
            { millimeter_t(std::stod(getDefaultField(0))),
              millimeter_t(std::stod(getDefaultField(1))),
              millimeter_t(std::stod(getDefaultField(2))) },
            degree_t(std::stod(getDefaultField(3))),
            m_Path / getDefaultField(4),
            gridPosition,
            std::move(extraFields)
        };
        m_Entries.emplace_back(std::move(entry));
    }

    return true;
}

bool
ImageDatabase::readDirectoryEntries()
{
    if (!m_Path.exists()) {
        return false;
    }

    // For reading contents of directory
    tinydir_dir dir;
    memset(&dir, 0, sizeof(dir));
    BOB_ASSERT(tinydir_open(&dir, m_Path.str().c_str()) == 0);
    for (; dir.has_next; BOB_ASSERT(tinydir_next(&dir) == 0)) {
        tinydir_file file;
        if (tinydir_readfile(&dir, &file)) {
            tinydir_close(&dir);
            throw std::runtime_error("Could not read from directory");
        }
        if (file.is_dir) {
            continue;
        }

        filesystem::path fileName = file.path;
        auto ext = fileName.extension();
        for (char &c : ext) {
            c = ::tolower(c);
        }
        if (ext == "jpg" || ext == "jpeg" || ext == "png") {
            // Save details to vector
            m_Entries.emplace_back();
            m_Entries.back().path = std::move(fileName);
        }
    }
    tinydir_close(&dir);

    // Try to sort by number in file name and fall back on alphabetic comparison
    const auto byName = [](const auto &e1, const auto &e2) {
        return ImageDatabase::fileNameCompare(e1.path.filename(), e2.path.filename());
    };
    std::sort(m_Entries.begin(), m_Entries.end(), byName);

    return !m_Entries.empty();
}

const std::tm &
ImageDatabase::getCreationTime() const
{
    // Check that we actually have a creation time
    BOB_ASSERT(m_CreationTime.tm_year != 0);
    return m_CreationTime;
}

//! Get the path of the directory corresponding to this ImageDatabase
const filesystem::path &
ImageDatabase::getPath() const
{
    return m_Path;
}

//! Get one Entry from the database
const ImageDatabase::Entry &
ImageDatabase::operator[](size_t i) const
{
    return m_Entries[i];
}

//! Start iterator for the database entries
std::vector<ImageDatabase::Entry>::const_iterator
ImageDatabase::begin() const
{
    return m_Entries.cbegin();
}

//! End iterator for the database entries
std::vector<ImageDatabase::Entry>::const_iterator
ImageDatabase::end() const
{
    return m_Entries.cend();
}

//! Number of entries in this database
size_t
ImageDatabase::size() const
{
    return m_Entries.size();
}

//! Check if there are any entries in this database
bool
ImageDatabase::empty() const
{
    return m_Entries.empty();
}

//! Check if the database is non-empty and a route-type database
bool
ImageDatabase::isRoute() const
{
    return !empty() && m_IsRoute;
}

//! Check if the database is non-empty and a grid-type database
bool
ImageDatabase::isGrid() const
{
    return !empty() && !m_IsRoute;
}

//! Load all of the images in this database into memory and return
std::vector<cv::Mat>
ImageDatabase::loadImages(const cv::Size &size, size_t frameSkip,
                          bool greyscale) const
{
    std::vector<cv::Mat> images;
    loadImages(images, size, frameSkip, greyscale);
    return images;
}

//! Load all of the images in this database into the specified std::vector<>
void
ImageDatabase::loadImages(std::vector<cv::Mat> &images, const cv::Size &size,
                          size_t frameSkip, bool greyscale) const
{
    size_t oldSize = images.size();
    images.resize(oldSize + m_Entries.size() / frameSkip);

    if (size == cv::Size{}) {
        forEachImage(
                [&images, oldSize](size_t i, const cv::Mat &img) {
                    img.copyTo(images[i + oldSize]);
                },
                frameSkip,
                greyscale);
    } else {
        forEachImage(
                [&images, oldSize, size](size_t i, const cv::Mat &img) {
                    cv::resize(img, images[i + oldSize], size);
                },
                frameSkip,
                greyscale);
    }
}

units::frequency::hertz_t
ImageDatabase::getFrameRate() const
{
    BOB_ASSERT(m_FrameRate != hertz_t{ 0 });
    return m_FrameRate;
}

//! Access the metadata for this database via OpenCV's persistence API
cv::FileNode
ImageDatabase::getMetadata() const
{
    BOB_ASSERT(hasMetadata());
    return m_MetadataYAML->operator[]("metadata");
}

//! Get the (directory) name of this database
std::string
ImageDatabase::getName() const
{
    return m_Path.filename();
}

//! Get the resolution of saved images
cv::Size
ImageDatabase::getResolution() const
{
    BOB_ASSERT(!m_Resolution.empty());
    return m_Resolution;
}

//! Check if this database has any saved metadata (yet)
bool
ImageDatabase::hasMetadata() const
{
    return static_cast<bool>(m_MetadataYAML);
}

/**!
 *  \brief Unwrap all the panoramic images in this database into a new
 *         folder, creating a new database.
 */
void
ImageDatabase::unwrap(const filesystem::path &destination, const cv::Size &unwrapRes) const
{
    // Check that the database doesn't already exist
    BOB_ASSERT(!(destination / EntriesFilename).exists());

    // Create object for unwrapping images
    std::string camName;
    getMetadata()["camera"]["name"] >> camName;
    ImgProc::OpenCVUnwrap360 unwrapper(getResolution(), unwrapRes, camName);
    filesystem::create_directory(destination);

    // Copy entries (CSV) file
    filesystem::copy_file(m_Path / EntriesFilename, destination / EntriesFilename);

    // Create new metadata (YAML) file
    {
        /*
         * There is no way to edit a persistence file in OpenCV, so we have
         * to do it ourselves in this ugly way.
         *
         * First, copy all fields, except for changing needsUnwrapping to false.
         */
        std::string line;
        std::ofstream ofs((destination / MetadataFilename).str());
        ofs.exceptions(std::ios::badbit | std::ios::failbit);
        std::ifstream ifs((m_Path / MetadataFilename).str());
        BOB_ASSERT(!ifs.fail());
        ifs.exceptions(std::ios::badbit);
        while (std::getline(ifs, line)) {
            const auto pos = line.find("needsUnwrapping:");
            if (pos != std::string::npos) {
                ofs << std::string(pos, ' ') << "needsUnwrapping: 0\n";
            } else {
                ofs << line << "\n";
            }
        }

        // Append info about the unwrapping object, indenting appropriately
        cv::FileStorage fs(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
        fs << "unwrapper" << unwrapper;
        std::stringstream ss;
        ss << fs.releaseAndGetString();
        std::getline(ss, line);
        std::getline(ss, line);
        while (std::getline(ss, line)) {
            ofs << "  " << line << "\n";
        }
    }

    /*
     * *If* forEachImage runs in parallel, then each thread will need its own
     * copy of these scrach variables.
     */
    static thread_local cv::Mat unwrapped;
    static thread_local std::string outPath;

    // Finally, unwrap all images and save to new folder
    forEachImage([&](size_t i, const cv::Mat &image) {
        unwrapper.unwrap(image, unwrapped);
        outPath = (destination / m_Entries[i].path.filename()).str();

        BOB_ASSERT(cv::imwrite(outPath, unwrapped));
    }, /*frameSkip=*/1, /*greyscale=*/false);
}

bool
ImageDatabase::fileNameCompare(const std::string &fn1, const std::string &fn2)
{
    // Find file names like (string)(number)(extension), e.g. "image45.png"
    std::regex regex{ R"(^([^0-9]*)([0-9]+)\.(.+)$)" };

    /*
     * If the prefixes (e.g. "image" in "image45.png") and the file extensions
     * match...
     */
    std::smatch m1, m2;
    if (std::regex_match(fn1, m1, regex) && std::regex_match(fn2, m2, regex)
        && m1[1] == m2[1] && m1[3] == m2[3]) {

        // ...then use the numbers for comparison
        return std::stoul(m1[2]) < std::stoul(m2[2]);
    }

    // ...else fall back on alphabetic comparison
    return fn1 < fn2;
}

void
ImageDatabase::loadMetadata()
{
    const auto metadataPath = m_Path / MetadataFilename;
    const bool metadataPresent = metadataPath.exists();
    if (!metadataPresent) {
        LOG_WARNING << "No " << MetadataFilename << " file found";
        m_IsRoute = true;
        m_MetadataYAML.reset();
    } else {
        std::ifstream ifs{ metadataPath.str() };
        ifs.exceptions(std::ios::badbit | std::ios::failbit);

        std::stringstream ss;
        ss << "%YAML:1.0\n"
           << ifs.rdbuf();

        // Parse metadata file
        m_MetadataYAML = std::make_unique<cv::FileStorage>(ss.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY);

        // What type of database is it?
        std::string dbtype;
        const auto metadata = getMetadata();
        metadata["type"] >> dbtype;
        if (dbtype == "route") {
            m_IsRoute = true;
        } else if (dbtype == "grid") {
            m_IsRoute = false;
        } else {
            throw std::runtime_error("Invalid database type \"" + dbtype + "\"");
        }

        // Get image resolution
        std::vector<int> size(2);
        metadata["camera"]["resolution"] >> size;
        m_Resolution = { size[0], size[1] };

        // These will only be set if database was recorded as a video file
        std::string videoFileName;
        metadata["video_file"] >> videoFileName;
        if (!videoFileName.empty()) {
            m_VideoFilePath = m_Path / videoFileName;
        }

        double fps = 0;
        metadata["frame_rate"] >> fps;
        m_FrameRate = hertz_t{ fps };

        std::string time;
        metadata["time"] >> time;
        if (!time.empty()) {
            std::istringstream ss{ time };
            ss >> std::get_time(&m_CreationTime, "%Y-%m-%d %H:%M:%S");
        }
    }
}
} // Navigation
} // BoB robotics
